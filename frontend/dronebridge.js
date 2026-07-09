const ROOT_URL = window.location.origin + "/"       // base URL only (no #hash / query) so sidebar nav anchors don't corrupt API request paths
// const ROOT_URL = "http://localhost:3000/"   // for testing with local json server
let conn_status = 0;		// connection status to the ESP32
let old_conn_status = 0;	// connection status before last update of UI to know when it changed
let serial_via_JTAG = 0;	// set to 1 if ESP32 is using the USB interface as serial interface for data and not using the UART. If 0 we set UART config to invisible for the user.
let last_byte_count = 0;
let last_timestamp_byte_count = 0;
let esp_chip_model = 0;		// according to get_esp_chip_model_str()
let recv_ser_bytes = 0;		// Total bytes received from serial interface
let serial_dec_mav_msgs = 0;	// Total MAVLink messages decoded from serial interface
let set_telem_proto = null;		// Telemetry protocol received by the ESP32
let active_sonar_source = 0;	// 0 none, 1 hardwired, 2 deeper
let cached_system_info = null;
let cached_runtime_info = null;
let cached_sonar_log_status = null;
const REQUEST_TIMEOUT_MS = 1000;

function is_abort_like_error(error) {
	if (!error) {
		return false;
	}
	if (error.name === "AbortError") {
		return true;
	}
	const message = (typeof error.message === "string") ? error.message.toLowerCase() : "";
	return message.includes("abort");
}

function normalize_request_error(error, fallback_message = "Request timed out. Please try again.") {
	if (error && error.isNormalizedTimeout === true) {
		return error;
	}
	if (is_abort_like_error(error)) {
		const timeout_error = new Error(fallback_message);
		timeout_error.isNormalizedTimeout = true;
		timeout_error.isSilentBackgroundError = true;
		return timeout_error;
	}
	return error;
}

function change_radio_dis_arm_visibility() {
	// we only support this feature when MAVLink or LTM are set AND when a standard Wi-Fi mode or BLE is enabled
	let radio_dis_onarm_div = document.getElementById("radio_dis_onarm_div")
	if ((document.getElementById("esp32_mode").value > "2" && document.getElementById("esp32_mode").value < "6") || document.getElementById("proto").value === "5") {
		radio_dis_onarm_div.style.display = "none";
	} else {
		radio_dis_onarm_div.style.display = "block";
	}
}

function change_ap_ip_visibility() {
	const esp32Mode = document.getElementById("esp32_mode").value;
	const elements = {
		ap_ip_div: document.getElementById("ap_ip_div"),
		ap_channel_div: document.getElementById("ap_channel_div"),
		lr_disclaimer_div: document.getElementById("esp-lr-ap-disclaimer"),
		ble_disclaimer_div: document.getElementById("ble_disclaimer_div"),
		wifi_ssid_div: document.getElementById("wifi_ssid_div"),
		wifi_en_gn_div: document.getElementById("wifi_en_gn_div"),
		static_ip_config_div: document.getElementById("static_ip_config_div"),
		pass_div: document.getElementById("pass_div"),
	};

	if (esp32Mode === "2") {
		elements.ap_ip_div.style.display = "none";
		elements.ap_channel_div.style.display = "none";
		elements.wifi_en_gn_div.style.display = "block";
		elements.static_ip_config_div.style.display = "block";
	} else {
		elements.ap_ip_div.style.display = "block";
		elements.ap_channel_div.style.display = "block";
		elements.wifi_en_gn_div.style.display = "none";
		elements.static_ip_config_div.style.display = "none";
	}

	if (esp32Mode === "6") {
		elements.ble_disclaimer_div.style.display = "block";
		elements.wifi_ssid_div.style.display = "none";
		elements.ap_channel_div.style.display = "none";
		elements.pass_div.style.display = "none";
		elements.ap_ip_div.style.display = "none";
	} else {
		elements.ble_disclaimer_div.style.display = "none";
		elements.wifi_ssid_div.style.display = "block";
		elements.pass_div.style.display = "block";
	}

	if (esp32Mode > "2" && esp32Mode < "6") {
		elements.lr_disclaimer_div.style.display = "block";
	} else {
		elements.lr_disclaimer_div.style.display = "none";
	}

	if (esp32Mode > "3" && esp32Mode < "6") {
		elements.ap_ip_div.style.display = "none";
		elements.wifi_ssid_div.style.visibility = "hidden";
	} else {
		elements.wifi_ssid_div.style.visibility = "visible";
	}
	change_radio_dis_arm_visibility();
}

function change_msp_ltm_visibility() {
	let msp_ltm_div = document.getElementById("msp_ltm_div");
	let trans_pack_size_div = document.getElementById("trans_pack_size_div");
	let rep_rssi_dbm_div = document.getElementById("rep_rssi_dbm_div");
	let telem_proto = document.getElementById("proto");
	if (telem_proto.value === "1") {
		msp_ltm_div.style.display = "block";
		trans_pack_size_div.style.display = "none";

	} else {
		msp_ltm_div.style.display = "none";
		trans_pack_size_div.style.display = "block";
	}
	if (telem_proto.value === "4") {
		rep_rssi_dbm_div.style.display = "block";
	} else {
		rep_rssi_dbm_div.style.display = "none";
	}
	change_radio_dis_arm_visibility();
}

function change_uart_visibility() {
	let tx_rx_div = document.getElementById("tx_rx_div");
	let rts_cts_div = document.getElementById("rts_cts_div");
	let rts_thresh_div = document.getElementById("rts_thresh_div");
	let baud_div = document.getElementById("baud_div");
	if (serial_via_JTAG === 0) {
		rts_cts_div.style.display = "block";
		tx_rx_div.style.display = "block";
		rts_thresh_div.style.display = "block";
		baud_div.style.display = "block";
	} else {
		rts_cts_div.style.display = "none";
		tx_rx_div.style.display = "none";
		rts_thresh_div.style.display = "none";
		baud_div.style.display = "none";
	}
}

function change_deeper_visibility() {
	let deeper_en = document.getElementById("ss_deeper_en").checked;
	let ssid_div = document.getElementById("ss_deeper_ssid_div");
	let pass_div = document.getElementById("ss_deeper_pass_div");
	let summary_div = document.getElementById("ss_deeper_summary_div");
	let debug_div = document.getElementById("ss_deeper_debug_div");
	if (deeper_en) {
		ssid_div.style.display = "block";
		pass_div.style.display = "block";
		summary_div.style.display = "block";
		debug_div.style.display = "block";
	} else {
		ssid_div.style.display = "none";
		pass_div.style.display = "none";
		summary_div.style.display = "none";
		debug_div.style.display = "none";
	}
}

function change_hardwired_visibility() {
	let hardwired_en = document.getElementById("ss_hardwired_en").checked;
	let hardwired_active = active_sonar_source === 1;
	let deeper_active = active_sonar_source === 2;
	let gpio_div = document.getElementById("sonar_gpio_div");
	let summary_div = document.getElementById("ss_hardwired_summary_div");
	let debug_div = document.getElementById("ss_hardwired_debug_div");
	if (!deeper_active && (hardwired_en || hardwired_active)) {
		gpio_div.style.display = "block";
		summary_div.style.display = "block";
		debug_div.style.display = "block";
	} else {
		gpio_div.style.display = "none";
		summary_div.style.display = "none";
		debug_div.style.display = "none";
		document.getElementById("ss_hardwired_depth").innerHTML = "Hardwired sonar disabled";
		document.getElementById("ss_hardwired_raw").innerHTML = "Hardwired sonar disabled";
		document.getElementById("ss_hardwired_filter_status").innerHTML = "Hardwired sonar disabled";
		document.getElementById("ss_hardwired_debug").value = "Hardwired sonar disabled.";
	}
}

function flow_control_check() {
	let gpio_rts = document.getElementById("gpio_rts");
	let gpio_cts = document.getElementById("gpio_cts");
	if (isNaN(gpio_rts.value) || isNaN(gpio_cts.value) || gpio_cts.value === '' || gpio_rts.value === '' || gpio_rts.value === gpio_cts.value) {
		show_toast("UART flow control disabled.")
	} else {
		show_toast("UART flow control enabled. Make sure RTS & CTS pins are connected!");
	}
}

/**
 * Convert a form into a JSON string
 * @param form The HTML form to convert
 * @returns {string} JSON formatted string
 */
function toJSONString(form) {
	let obj = {}
	let elements = form.querySelectorAll("input, select")
	for (let i = 0; i < elements.length; ++i) {
		let element = elements[i]
		let name = element.name;
		let value = element.value;
		if (name) {
			if (element.type === "checkbox") {
				// convert checked/not checked to 1 & 0 as value
				obj[name] = element.checked ? 1 : 0;
			} else if ((element.type === "number" || element.tagName === "SELECT") &&
				!isNaN(Number(value))) {
				obj[name] = parseInt(value)
			} else {
				// treat all text/password inputs as strings so numeric passwords
				// do not get coerced into numbers
				obj[name] = value
			}
		}
	}
	return JSON.stringify(obj)
}

/**
 * Request data from the ESP to display in the GUI
 * @param api_path API path/request path
 * @returns {Promise<any>}
 */
async function get_json(api_path) {
	let req_url = ROOT_URL + api_path;

	const controller = new AbortController();
	const timeout = setTimeout(() => {
		controller.abort();
	}, REQUEST_TIMEOUT_MS);

	try {
		const response = await fetch(req_url, {
			signal: controller.signal
		});
		if (!response.ok) {
			const message = `An error has occured: ${response.status}`;
			conn_status = 0;
			throw new Error(message);
		}
		return await response.json();
	} catch (error) {
		conn_status = 0;
		throw normalize_request_error(error);
	} finally {
		clearTimeout(timeout);
	}
}

/**
 * Create a response with JSON data attached
 * @param api_path API URL path
 * @param json_data JSON body data to send
 * @returns {Promise<any>}
 */
async function send_json(api_path, json_data = undefined) {
	let post_url = ROOT_URL + api_path;
	const response = await fetch(post_url, {
		method: 'POST',
		headers: {
			'Accept': 'application/json',
			'Content-Type': 'application/json',
			"charset": 'utf-8'
		},
		body: json_data
	});
	if (!response.ok) {
		conn_status = 0
		const message = `An error has occured: ${response.status}`;
		throw new Error(message);
	}
	return await response.json();
}

async function get_text(api_path) {
	let get_url = ROOT_URL + api_path;
	const controller = new AbortController();
	const timeout = setTimeout(() => {
		controller.abort();
	}, REQUEST_TIMEOUT_MS * 5);

	try {
		const response = await fetch(get_url, {
			method: 'GET',
			signal: controller.signal
		});
		if (!response.ok) {
			conn_status = 0;
			const body = await response.text();
			throw new Error(body || `An error has occured: ${response.status}`);
		}
		return await response.text();
	} catch (error) {
		throw normalize_request_error(error, "Timed out while reading the persistent sonar log. Please try again.");
	} finally {
		clearTimeout(timeout);
	}
}

function get_esp_chip_model_str(esp_model_index) {
	switch (esp_model_index) {
		default:
		case 0:
			return "unknown/unsupported ESP32 chip";
		case 1:
			return "ESP32";
		case 2:
			return "ESP32-S2";
		case 9:
			return "ESP32-S3";
		case 5:
			return "ESP32-C3";
		case 13:
			return "ESP32-C6";
		case 12:
			return "ESP32-C5";
	}
}

function render_about_text() {
	let branding_line = "DBB Companion v0.1 - waiting for a response from the ESP32";
	let runtime_line = "Running firmware version: waiting for OTA info from the ESP32";

	if (cached_system_info !== null) {
		branding_line = "DBB Companion v0.1 Forked from DroneBridge for ESP32 v" + cached_system_info["major_version"] +
			"." + cached_system_info["minor_version"] + "." + cached_system_info["patch_version"] + " (" + cached_system_info["maturity_version"] + ")" +
			" - esp-idf " + cached_system_info["idf_version"] + " - " + get_esp_chip_model_str(cached_system_info["esp_chip_model"]);
	}

	if (cached_runtime_info !== null) {
		runtime_line = "Running firmware v" + cached_runtime_info["running_app_version"] +
			" (" + cached_runtime_info["running_app_date"] + " " + cached_runtime_info["running_app_time"] + ")" +
			" - " + cached_runtime_info["running_partition_label"];
	}

	document.getElementById("about").innerHTML = "&reg;Danevi Bait Boats v0.1 " + (cached_runtime_info !== null ? cached_runtime_info["running_partition_label"] : "ota_x") + " &middot; Valentin Danev &middot; Valentin@danevi.info &middot; +359 882 619 117" + "<span class=\"credit\">Built on <a href=\"https://github.com/DroneBridge/ESP32\" target=\"_blank\" rel=\"noopener\">DroneBridge for ESP32</a> (Apache-2.0) by Wolfgang Christl</span>";
}

function render_sonar_log_status() {
	let status_div = document.getElementById("sonar_log_status");
	if (status_div == null) {
		return;
	}

	if (cached_sonar_log_status === null || !cached_sonar_log_status["mounted"]) {
		status_div.innerHTML = "Persistent sonar log unavailable. This is expected until the logger build with the dedicated logs partition is flashed once by cable.";
		return;
	}

	const logBytes = parseInt(cached_sonar_log_status["log_file_bytes"]);
	const maxLogBytes = parseInt(cached_sonar_log_status["max_log_file_bytes"]);
	const usedBytes = parseInt(cached_sonar_log_status["partition_used_bytes"]);
	const totalBytes = parseInt(cached_sonar_log_status["partition_total_bytes"]);
	const compactionCount = parseInt(cached_sonar_log_status["compaction_count"]);
	const percent = maxLogBytes > 0 ? ((logBytes / maxLogBytes) * 100).toFixed(1) : "0.0";

	status_div.innerHTML = "Persistent log file: " + logBytes + " / " + maxLogBytes + " bytes (" + percent +
		"% of rolling budget)<br>Partition usage: " + usedBytes + " / " + totalBytes +
		" bytes, compactions this boot: " + compactionCount;
}

function get_sonar_log_status() {
	get_json("api/logs/status").then(json_data => {
		cached_sonar_log_status = json_data;
		render_sonar_log_status();
	}).catch(error => {
		const normalized_error = normalize_request_error(error);
		document.getElementById("sonar_log_status").innerHTML = normalized_error.message;
	});
}

async function refresh_sonar_log(quiet = false) {
	try {
		document.getElementById("sonar_persistent_log").value = "Loading persistent sonar log...";
		const log_text = await get_text("api/logs/sonar");
		document.getElementById("sonar_persistent_log").value = log_text;
		get_sonar_log_status();
	} catch (error) {
		document.getElementById("sonar_persistent_log").value = error.message;
		if (!quiet) {
			show_toast(error.message, "#7a0f19");
		}
	}
}

function download_sonar_log() {
	window.location = ROOT_URL + "api/logs/sonar/download";
}

async function clear_sonar_log() {
	if (confirm("Do you want to clear the persistent sonar log?\nThis removes the currently saved rolling history.") !== true) {
		return;
	}

	const post_url = ROOT_URL + "api/logs/sonar";
	const response = await fetch(post_url, {
		method: 'DELETE',
		headers: {
			'Accept': 'application/json',
			'Content-Type': 'application/json',
			"charset": 'UTF-8'
		},
		body: null
	});

	if (!response.ok) {
		const message = await response.text();
		show_toast(message || `An error has occured: ${response.status}`, "#7a0f19");
		return;
	}

	const json = await response.json();
	document.getElementById("sonar_persistent_log").value = "Persistent sonar log cleared.";
	show_toast(json["msg"]);
	get_sonar_log_status();
}

function get_runtime_firmware_info() {
	get_json("api/update/info").then(json_data => {
		cached_runtime_info = json_data;
		render_about_text();
	}).catch(error => {
		conn_status = 0
		error.message;
		return -1;
	});
	return 0;
}

function get_system_info() {
	get_json("api/system/info").then(json_data => {
		console.log("Received settings: " + json_data)
		cached_system_info = json_data;
		render_about_text();
		serial_via_JTAG = json_data["serial_via_JTAG"];
		get_runtime_firmware_info();
		// set external antenna option visible based on info if RF switch is available on the board
		if (parseInt(json_data["has_rf_switch"]) === 1) {
			document.getElementById("ant_use_ext_div").style.display = "block";
		} else {
			document.getElementById("ant_use_ext_div").style.display = "none";
		}
	}).catch(error => {
		conn_status = 0
		error.message;
		return -1;
	});
	return 0;
}

function update_conn_status() {
	if (conn_status)
		document.getElementById("web_conn_status").innerHTML = "<span class=\"dot_green\"></span> connected to Companion"
	else {
		document.getElementById("web_conn_status").innerHTML = "<span class=\"dot_red\"></span> disconnected from Companion"
		document.getElementById("current_client_ip").innerHTML = ""
	}
	if (conn_status !== old_conn_status) {
		// connection status changed. Update settings and UI
		get_system_info();
		get_settings();
		get_sonar_log_status();
		const sonar_log_elem = document.getElementById("sonar_persistent_log");
		if (sonar_log_elem != null &&
			(sonar_log_elem.value === "Waiting for persistent sonar log..." ||
			 sonar_log_elem.value === "Loading persistent sonar log...")) {
			sonar_log_elem.value = "Press Read / Refresh Log to load the persistent sonar log.";
		}
		setTimeout(change_msp_ltm_visibility, 500);
		setTimeout(change_ap_ip_visibility, 500);
		setTimeout(change_uart_visibility, 500);
		setTimeout(change_hardwired_visibility, 500);
		setTimeout(change_deeper_visibility, 500);
	}
	old_conn_status = conn_status
}

/**
 * Get connection status information and display it in the GUI
 */
function get_stats() {
	get_json("api/system/stats").then(json_data => {
		conn_status = 1
		let d = new Date();
		recv_ser_bytes = parseInt(json_data["read_bytes"]);
		serial_dec_mav_msgs = parseInt(json_data["serial_dec_mav_msgs"]);
		let bytes_per_second = 0;
		let current_time = d.getTime();
		if (last_byte_count > 0 && last_timestamp_byte_count > 0 && !isNaN(recv_ser_bytes)) {
			bytes_per_second = (recv_ser_bytes - last_byte_count) / ((current_time - last_timestamp_byte_count) / 1000);
		}
		last_timestamp_byte_count = current_time;
		if (!isNaN(recv_ser_bytes) && recv_ser_bytes > 1000000) {
			document.getElementById("read_bytes").innerHTML = (recv_ser_bytes / 1000000).toFixed(3) + " MB (" + ((bytes_per_second * 8) / 1000).toFixed(2) + " kbit/s)"
		} else if (!isNaN(recv_ser_bytes) && recv_ser_bytes > 1000) {
			document.getElementById("read_bytes").innerHTML = (recv_ser_bytes / 1000).toFixed(2) + " kB (" + ((bytes_per_second * 8) / 1000).toFixed(2) + " kbit/s)"
		} else if (!isNaN(recv_ser_bytes)) {
			document.getElementById("read_bytes").innerHTML = recv_ser_bytes + " bytes (" + Math.round(bytes_per_second) + " byte/s)"
		}
		last_byte_count = recv_ser_bytes;

		let tcp_clients = parseInt(json_data["tcp_connected"])
		if (!isNaN(tcp_clients) && tcp_clients === 1) {
			document.getElementById("tcp_connected").innerHTML = tcp_clients + " client"
		} else if (!isNaN(tcp_clients)) {
			document.getElementById("tcp_connected").innerHTML = tcp_clients + " clients"
		}
		// UDP clients for tooltip
		let udp_clients_string = ""
		if (json_data.hasOwnProperty("udp_clients")) {
			let udp_conn_jsonarray = json_data["udp_clients"];
			for (let i = 0; i < udp_conn_jsonarray.length; i++) {
				udp_clients_string = udp_clients_string + udp_conn_jsonarray[i];
				if ((i + 1) !== udp_conn_jsonarray.length) {
					udp_clients_string = udp_clients_string + "<br>";
				}
			}
			if (udp_conn_jsonarray.length === 0) {
				udp_clients_string = "-";
			} else {
				document.getElementById("tooltip_udp_clients").innerHTML = udp_clients_string;
			}
		}

		let udp_clients = parseInt(json_data["udp_connected"])
		if (!isNaN(udp_clients) && udp_clients === 1) {
			document.getElementById("udp_connected").innerHTML = "<span class=\"tooltiptext\" id=\"tooltip_udp_clients\">" + udp_clients_string + "</span>" + udp_clients + " client"
		} else if (!isNaN(udp_clients)) {
			document.getElementById("udp_connected").innerHTML = "<span class=\"tooltiptext\" id=\"tooltip_udp_clients\">" + udp_clients_string + "</span>" + udp_clients + " clients"
		}

		if ('esp_rssi' in json_data) {
			let rssi = parseInt(json_data["esp_rssi"])
			if (!isNaN(rssi) && rssi < 0) {
				document.getElementById("current_client_ip").innerHTML = "IP Address: " + json_data["current_client_ip"] + "<br />Signal Strength: " + rssi + "dBm"
			} else if (!isNaN(rssi)) {
				document.getElementById("current_client_ip").innerHTML = "IP Address: " + json_data["current_client_ip"]
			}
		} else if ('connected_sta' in json_data) {
			let a = ""
			json_data["connected_sta"].forEach((item) => {
				a = a + "Client: " + item.sta_mac + " Signal Strength: " + item.sta_rssi + "dBm<br />"
			});
			document.getElementById("current_client_ip").innerHTML = a
		}

		if ('deeper_debug' in json_data) {
		document.getElementById("ss_deeper_debug").value = json_data["deeper_debug"];
		}
		if ('hardwired_debug' in json_data) {
			document.getElementById("ss_hardwired_debug").value = json_data["hardwired_debug"];
		}
		if ('active_sonar_source' in json_data) {
			active_sonar_source = parseInt(json_data["active_sonar_source"]);
		}
		change_hardwired_visibility();
		update_hardwired_readouts(json_data);
		update_deeper_readouts(json_data);

	}).catch(error => {
		conn_status = 0
		error.message;
	});
}

function update_hardwired_readouts(json_data) {
	if (!document.getElementById("ss_hardwired_en").checked && active_sonar_source !== 1) {
		document.getElementById("ss_hardwired_depth").innerHTML = "Hardwired sonar disabled";
		document.getElementById("ss_hardwired_raw").innerHTML = "Hardwired sonar disabled";
		document.getElementById("ss_hardwired_filter_status").innerHTML = "Hardwired sonar disabled";
		return;
	}

	let depth_mm = parseInt(json_data["hardwired_depth_mm"]);
	let sample_age_ms = parseInt(json_data["hardwired_sample_age_ms"]);
	let raw_depth_mm = parseInt(json_data["hardwired_raw_depth_mm"]);
	let raw_sample_age_ms = parseInt(json_data["hardwired_raw_sample_age_ms"]);
	let last_good_depth_mm = parseInt(json_data["hardwired_last_good_depth_mm"]);
	let last_good_sample_age_ms = parseInt(json_data["hardwired_last_good_sample_age_ms"]);
	let zero_run_active = parseInt(json_data["hardwired_zero_run_active"]) === 1;
	let zero_run_age_ms = parseInt(json_data["hardwired_zero_run_age_ms"]);
	let consecutive_zero_frames = parseInt(json_data["hardwired_consecutive_zero_frames"]);
	let zero_filter_holding = parseInt(json_data["hardwired_zero_filter_holding"]) === 1;

	if (!isNaN(depth_mm) && depth_mm >= 0) {
		let ageSuffix = (!isNaN(sample_age_ms) && sample_age_ms >= 0) ? " (" + sample_age_ms + " ms ago)" : "";
		document.getElementById("ss_hardwired_depth").innerHTML =
			(depth_mm / 1000).toFixed(2) + " m" + ageSuffix;
	} else {
		document.getElementById("ss_hardwired_depth").innerHTML =
			zero_run_active ? "Suppressed during zero run" : "Waiting for UART frame";
	}

	if (!isNaN(raw_depth_mm) && raw_depth_mm >= 0) {
		let rawAgeSuffix = (!isNaN(raw_sample_age_ms) && raw_sample_age_ms >= 0) ? " (" + raw_sample_age_ms + " ms ago)" : "";
		document.getElementById("ss_hardwired_raw").innerHTML =
			(raw_depth_mm / 1000).toFixed(2) + " m" + rawAgeSuffix;
	} else {
		document.getElementById("ss_hardwired_raw").innerHTML = "No raw frame yet";
	}

	let lastGoodText = (!isNaN(last_good_depth_mm) && last_good_depth_mm >= 0)
		? (last_good_depth_mm / 1000).toFixed(2) + " m"
		: "none yet";
	if (!isNaN(last_good_sample_age_ms) && last_good_sample_age_ms >= 0 &&
		!isNaN(last_good_depth_mm) && last_good_depth_mm >= 0) {
		lastGoodText += " (" + last_good_sample_age_ms + " ms ago)";
	}

	let filterStatus = "Waiting for first good reading";
	if (zero_filter_holding) {
		let holdAgeSuffix = (!isNaN(zero_run_age_ms) && zero_run_age_ms >= 0)
			? " for " + zero_run_age_ms + " ms"
			: "";
		let zeroCountSuffix = (!isNaN(consecutive_zero_frames) && consecutive_zero_frames > 0)
			? " after " + consecutive_zero_frames + " zero frame(s)"
			: "";
		filterStatus = "Holding last good " + lastGoodText + holdAgeSuffix + zeroCountSuffix;
	} else if (zero_run_active) {
		let suppressAgeSuffix = (!isNaN(zero_run_age_ms) && zero_run_age_ms >= 0)
			? " (" + zero_run_age_ms + " ms)"
			: "";
		let suppressCountSuffix = (!isNaN(consecutive_zero_frames) && consecutive_zero_frames > 0)
			? ", " + consecutive_zero_frames + " zero frame(s)"
			: "";
		filterStatus = "Suppressing zero run" + suppressAgeSuffix + suppressCountSuffix + ". Last good: " + lastGoodText;
	} else if (!isNaN(depth_mm) && depth_mm >= 0) {
		filterStatus = "Live reading";
	} else if (!isNaN(raw_depth_mm) && raw_depth_mm === 0) {
		filterStatus = "Zero frame seen before first good reading";
	}
	document.getElementById("ss_hardwired_filter_status").innerHTML = filterStatus;
}

function update_deeper_readouts(json_data) {
	let depth_mm = parseInt(json_data["deeper_depth_mm"]);
	let temperature_c_tenths = parseInt(json_data["deeper_temp_c_tenths"]);
	let satellites = parseInt(json_data["deeper_satellites"]);
	let gps_fix = parseInt(json_data["deeper_gps_fix"]) === 1;
	let has_coordinates = parseInt(json_data["deeper_has_coordinates"]) === 1;
	let latitude_deg = parseFloat(json_data["deeper_latitude_deg"]);
	let longitude_deg = parseFloat(json_data["deeper_longitude_deg"]);
	let sample_age_ms = parseInt(json_data["deeper_sample_age_ms"]);

	if (!isNaN(depth_mm) && depth_mm >= 0) {
		document.getElementById("ss_deeper_depth").innerHTML =
			(depth_mm / 1000).toFixed(2) + " m";
	} else {
		document.getElementById("ss_deeper_depth").innerHTML = "Waiting for $SDDBT";
	}

	if (!isNaN(temperature_c_tenths) && temperature_c_tenths > -100000) {
		document.getElementById("ss_deeper_temp").innerHTML =
			(temperature_c_tenths / 10).toFixed(1) + " C";
	} else {
		document.getElementById("ss_deeper_temp").innerHTML = "Waiting for $YXMTW";
	}

	if (!isNaN(satellites) && satellites >= 0) {
		let gpsStatus = gps_fix ? "fix" : "no fix";
		let ageSuffix = (!isNaN(sample_age_ms) && sample_age_ms >= 0) ? " (" + sample_age_ms + " ms ago)" : "";
		document.getElementById("ss_deeper_satellites").innerHTML =
			satellites + " satellites, " + gpsStatus + ageSuffix;
	} else {
		document.getElementById("ss_deeper_satellites").innerHTML = "Waiting for $GNGGA";
	}

	if (has_coordinates && !isNaN(latitude_deg) && !isNaN(longitude_deg)) {
		document.getElementById("ss_deeper_coordinates").innerHTML =
			"Lat " + latitude_deg.toFixed(6) + "<br>Lon " + longitude_deg.toFixed(6);
	} else {
		document.getElementById("ss_deeper_coordinates").innerHTML = "Waiting for valid fix";
	}
}

/**
 * Get settings from ESP and display them in the GUI. JSON objects have to match the element ids
 *  returns 0 on success and -1 on failure
 */
function get_settings() {
	get_json("api/settings").then(json_data => {
		console.log("Received settings: " + json_data)
		conn_status = 1
		for (const key in json_data) {
			if (json_data.hasOwnProperty(key)) {
				let elem = document.getElementById(key)
				if (elem != null) {
					if (elem.type === "checkbox") {
						// translate 1 & 0 to checked and not checked
						elem.checked = json_data[key] === 1;
					} else {
						elem.value = json_data[key] + ""
					}
				}
			}
		}
		set_telem_proto = document.getElementById("proto").value;
		change_hardwired_visibility();
	}).catch(error => {
		conn_status = 0;
		const normalized_error = normalize_request_error(error);
		if (normalized_error.isSilentBackgroundError !== true) {
			show_toast(normalized_error.message);
		}
		return -1;
	});
	change_ap_ip_visibility();
	change_msp_ltm_visibility();
	change_hardwired_visibility();
	change_deeper_visibility();
	return 0;
}

function add_new_udp_client() {
	let ip = prompt("Please enter the IP address of the UDP receiver", "192.168.2.X");
	if (ip == null) {
		show_toast("Operation cancelled by user.");
		return;
	}
	let port = prompt("Please enter the port number of the UDP receiver", "14550");
	if (port == null) {
		show_toast("Operation cancelled by user.")
		return;
	}
	port = parseInt(port);
	let save_to_nvm = confirm("Save this UDP client to the permanent storage so it will be auto added after reboot/reset?\nYou can only save one UDP client to the memory. The old ones will be overwritten.\nSelect no if you only want to add this client for this session.");
	const ippattern = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;

	if (ip != null && port != null && ippattern.test(ip)) {
		let myjson = {
			udp_client_ip: ip,
			udp_client_port: port,
			save: save_to_nvm
		};
		send_json("api/settings/clients/udp", JSON.stringify(myjson)).then(send_response => {
			console.log(send_response);
			conn_status = 1
			show_toast(send_response["msg"])
		}).catch(error => {
			show_toast(error.message);
		});
	} else {
		show_toast("Error: Enter valid IP and port!")
	}
}

async function clear_udp_clients() {
	if (confirm("Do you want to remove all UDP connections?\nGCS will have to re-connect.") === true) {
		let post_url = ROOT_URL + "api/settings/clients/clear_udp";
		const response = await fetch(post_url, {
			method: 'DELETE',
			headers: {
				'Accept': 'application/json',
				'Content-Type': 'application/json',
				"charset": 'UTF-8'
			},
			body: null
		});
		if (!response.ok) {
			conn_status = 0
			const message = `An error has occured: ${response.status}`;
			throw new Error(message);
		}
	} else {
		// cancel
	}
}

function show_toast(msg, background_color = "#0058a6") {
	Toastify({
		text: msg,
		duration: 5000,
		newWindow: true,
		close: true,
		gravity: "top", // `top` or `bottom`
		position: "center", // `left`, `center` or `right`
		style: {
			background: background_color,
			color: "#ff9734",
			borderColor: "#ff9734",
			borderStyle: "solid",
			borderRadius: "2px",
			borderWidth: "1px",
		},
		stopOnFocus: true, // Prevents dismissing of toast on hover
	}).showToast();
}

function check_validity() {
	let valid = true;
	let wifi_pass = document.getElementById("wifi_pass")
	if (!wifi_pass.checkValidity()) {
		show_toast("Error: 8<(password length)<64");
		valid = false;
	}
	let deeper_pass = document.getElementById("ss_deeper_pass")
	if (document.getElementById("ss_deeper_en").checked &&
		deeper_pass.value.length > 0 &&
		deeper_pass.value.length < 8) {
		show_toast("Error: Deeper password must be empty or at least 8 characters.");
		valid = false;
	}
	return valid;
}

function check_for_issues() {
	let issue_div = document.getElementById("issue_div");
	if (set_telem_proto === "4" && serial_dec_mav_msgs === 0 && recv_ser_bytes !== 0) {
		issue_div.style.display = "block";
	} else {
		issue_div.style.display = "none";
	}
}

function save_settings() {
	let form = document.getElementById("settings_form")
	if (check_validity()) {
		let json_data = toJSONString(form)
		send_json("api/settings", json_data).then(send_response => {
			console.log(send_response);
			conn_status = 1
			show_toast(send_response["msg"])
			get_settings()  // update UI with new settings
		}).catch(error => {
			show_toast(error.message);
		});
	} else {
		console.log("Form was not filled out correctly.")
	}
}
