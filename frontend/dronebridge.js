const ROOT_URL = window.location.origin + "/"       // base URL only (no #hash / query) so sidebar nav anchors don't corrupt API request paths
// const ROOT_URL = "http://localhost:3000/"   // for testing with local json server
let conn_status = 0;		// connection status to the ESP32
let old_conn_status = 0;	// connection status before last update of UI to know when it changed
// The boat's Wi-Fi link drops the occasional packet (hull-mounted ESP, 802.11b).
// One lost poll is normal and must not be reported as "disconnected": every flip
// of conn_status makes update_conn_status() fire four extra requests, which on a
// lossy link provokes the next drop. Only call the link down after this many
// consecutive failures - a real outage fails every poll and still trips quickly.
const CONN_FAIL_THRESHOLD = 3;
let conn_fail_count = 0;
let serial_via_JTAG = 0;	// set to 1 if ESP32 is using the USB interface as serial interface for data and not using the UART. If 0 we set UART config to invisible for the user.
let last_byte_count = 0;
let last_sent_byte_count = 0;
let last_timestamp_byte_count = 0;
let esp_chip_model = 0;		// according to get_esp_chip_model_str()
let recv_ser_bytes = 0;		// Total bytes received from serial interface
let sent_ser_bytes = 0;		// Total bytes successfully written to serial interface
let serial_dec_mav_msgs = 0;	// Total MAVLink messages decoded from serial interface
let set_telem_proto = null;		// Telemetry protocol received by the ESP32
let active_sonar_source = 0;	// 0 none, 1 hardwired, 2 deeper
let cached_system_info = null;
let cached_runtime_info = null;
let cached_sonar_log_status = null;
let cached_log_sessions = [];
let esp_uptime_ms = null;
// 1000 ms was too tight: a single TCP retransmit on the boat link exceeds it.
// get_stats() polls every 500 ms, so a longer timeout could stack requests -
// s_stats_in_flight below skips a tick while one is still outstanding.
const REQUEST_TIMEOUT_MS = 2500;
let s_stats_in_flight = false;
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
			} else if ((element.type === "number" || element.tagName === "SELECT" ||
				element.dataset.number !== undefined) && !isNaN(Number(value))) {
				// data-number marks inputs that are numeric on the ESP but not
				// type="number" in the form (hidden fields). Without it they are
				// sent as text, cJSON reads valueint as 0, and the ESP rejects
				// the value.
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
			headers: {},
			signal: controller.signal
		});
		if (!response.ok) {
			const message = `An error has occured: ${response.status}`;
			throw new Error(message);
		}
		return await response.json();
	} catch (error) {
		// Counted here and nowhere else, so one failed request is one failure.
		// Callers used to set conn_status in their own catch as well, which
		// double counted every drop.
		note_conn_fail();
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
		// Not a connection problem: the board answered, it just refused the
		// request (e.g. 400 for an out-of-range setting). Marking the link down
		// here would flash "disconnected" every time a value is rejected.
		note_conn_ok();
		// The ESP explains refusals in the body - e.g. which setting was out of
		// range and what the allowed range is. Show that instead of a bare
		// status code, otherwise a rejected value looks like "it just did not save".
		let message = `An error has occured: ${response.status}`;
		try {
			const body = await response.json();
			if (body && body.msg) {
				message = body.msg;
			}
		} catch (e) {
			// body was not JSON - keep the status code message
		}
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
			headers: {},
			signal: controller.signal
		});
		if (!response.ok) {
			note_conn_ok(); // the board answered - the link is up
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
	let branding_line = "DBB Companion v0.2 - waiting for a response from the ESP32";
	let runtime_line = "Running firmware version: waiting for OTA info from the ESP32";

	if (cached_system_info !== null) {
		branding_line = "DBB Companion v0.2 Forked from DroneBridge for ESP32 v" + cached_system_info["major_version"] +
			"." + cached_system_info["minor_version"] + "." + cached_system_info["patch_version"] + " (" + cached_system_info["maturity_version"] + ")" +
			" - esp-idf " + cached_system_info["idf_version"] + " - " + get_esp_chip_model_str(cached_system_info["esp_chip_model"]);
	}

	if (cached_runtime_info !== null) {
		runtime_line = "Running firmware v" + cached_runtime_info["running_app_version"] +
			" (" + cached_runtime_info["running_app_date"] + " " + cached_runtime_info["running_app_time"] + ")" +
			" - " + cached_runtime_info["running_partition_label"];
	}

	document.getElementById("about").innerHTML = "&reg;Danevi Bait Boats v0.2 " + (cached_runtime_info !== null ? cached_runtime_info["running_partition_label"] : "ota_x") + " &middot; Valentin Danev &middot; Valentin@danevi.info &middot; +359 882 619 117" + "<span class=\"credit\">Built on <a href=\"https://github.com/DroneBridge/ESP32\" target=\"_blank\" rel=\"noopener\">DroneBridge for ESP32</a> (Apache-2.0) by Wolfgang Christl</span>";
}

function update_fc_status_chips(json_data) {
	let armed_chip = document.getElementById("fc_armed_status");
	let mode_chip = document.getElementById("fc_mode_status");
	if (armed_chip == null || mode_chip == null) {
		return;
	}

	let fc_seen = parseInt(json_data["fc_seen"]) === 1;
	let fc_stale = parseInt(json_data["fc_stale"]) === 1;
	if (!fc_seen) {
		armed_chip.innerHTML = "no FC";
		mode_chip.innerHTML = "no heartbeat";
		return;
	}

	if (fc_stale) {
		armed_chip.innerHTML = "stale";
		mode_chip.innerHTML = "stale";
		return;
	}

	let fc_armed = parseInt(json_data["fc_armed"]) === 1;
	let fc_mode = json_data["fc_mode"] || "unknown";
	let custom_mode = parseInt(json_data["fc_custom_mode"]);
	armed_chip.innerHTML = fc_armed ? "YES" : "no";
	mode_chip.innerHTML = fc_mode === "unknown" && !isNaN(custom_mode)
		? "unknown " + custom_mode
		: fc_mode;
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

	const usedBytes = parseInt(cached_sonar_log_status["partition_used_bytes"]);
	const totalBytes = parseInt(cached_sonar_log_status["partition_total_bytes"]);
	const freeBytes = parseInt(cached_sonar_log_status["partition_free_bytes"]);
	const sessionActive = cached_sonar_log_status["session_active"] === true;
	const activeId = parseInt(cached_sonar_log_status["active_session_id"]);
	const manualMs = parseInt(cached_sonar_log_status["manual_remaining_ms"]);
	const reserve = parseInt(cached_sonar_log_status["fc_image_reserve_bytes"]);
	const headroom = parseInt(cached_sonar_log_status["filesystem_headroom_bytes"]);
	status_div.innerHTML = "Partition: " + usedBytes + " used / " + totalBytes + " bytes (" + freeBytes + " free)" +
		"<br>Protected: " + reserve + " bytes for FC firmware + " + headroom + " bytes filesystem headroom" +
		"<br>Capture: " + (sessionActive ? "session " + activeId + " active" : "waiting for arm or manual start") +
		(manualMs > 0 ? ", manual timer " + Math.ceil(manualMs / 60000) + " min remaining" : "") +
		"; complete sessions: " + parseInt(cached_sonar_log_status["completed_session_count"]);
}

let s_log_status_in_flight = false;
function get_sonar_log_status() {
	// Polled every 5 s from index.html: never stack requests, and stop
	// entirely while the tab is hidden - this ESP's HTTP server starves the
	// MAVLink bridge under background polling.
	if (document.hidden || s_log_status_in_flight) return;
	s_log_status_in_flight = true;
	get_json("api/logs/status").then(json_data => {
		cached_sonar_log_status = json_data;
		render_sonar_log_status();
	}).catch(error => {
		const normalized_error = normalize_request_error(error);
		document.getElementById("sonar_log_status").innerHTML = normalized_error.message;
	}).finally(() => {
		s_log_status_in_flight = false;
	});
}

function refresh_log_sessions() {
	get_json("api/logs/sessions").then(data => {
		cached_log_sessions = Array.isArray(data.sessions) ? data.sessions : [];
		const select = document.getElementById("log_session_select");
		if (select == null) return;
		const previous = select.value;
		select.innerHTML = "";
		if (cached_log_sessions.length === 0) {
			// Keep the placeholder alive - an empty <select> looks broken and
			// would let session=0 requests through.
			const option = document.createElement("option");
			option.value = "";
			option.textContent = "No captured sessions yet";
			select.appendChild(option);
			return;
		}
		cached_log_sessions.forEach(session => {
			const option = document.createElement("option");
			option.value = session.id;
			option.textContent = "Session " + String(session.id).padStart(7, "0") +
				(session.active ? " (active)" : "") + " - trip " + session.trip_bytes +
				" B, hardwired " + session.hardwired_bytes + " B, Deeper " + session.deeper_bytes + " B";
			select.appendChild(option);
		});
		if (previous && Array.from(select.options).some(option => option.value === previous)) select.value = previous;
	}).catch(error => {
		// Background refresh (conn-status flips call this too): a listing
		// failure must never clobber the log view the user is reading.
		// get_json already counted the failure; leave the list as it was.
	});
}

function selected_log_url(download = false) {
	const type = document.getElementById("log_type_select").value;
	const session = document.getElementById("log_session_select").value;
	const needs_session = type === "trip" || type === "hardwired" || type === "deeper";
	// Session-scoped streams without a real session would only produce a 400
	// from the firmware - refuse client-side with a usable message instead.
	if (needs_session && !parseInt(session)) return null;
	return "api/logs/file?type=" + encodeURIComponent(type) + "&session=" + encodeURIComponent(session || "0") +
		(download ? "&download=1" : "");
}

async function refresh_sonar_log(quiet = false) {
	const url = selected_log_url(false);
	if (url == null) {
		document.getElementById("sonar_persistent_log").value = "Select a capture session for this log type.";
		return;
	}
	try {
		document.getElementById("sonar_persistent_log").value = "Loading selected log...";
		const log_text = await get_text(url);
		document.getElementById("sonar_persistent_log").value = log_text;
		get_sonar_log_status();
		refresh_log_sessions();
	} catch (error) {
		document.getElementById("sonar_persistent_log").value = error.message;
		if (!quiet) {
			show_toast(error.message, "#7a0f19");
		}
	}
}

function download_sonar_log() {
	const url = selected_log_url(true);
	if (url == null) {
		show_toast("Select a capture session for this log type.", "#7a0f19");
		return;
	}
	window.location = ROOT_URL + url;
}

async function logging_delete(api_path) {
	const post_url = ROOT_URL + api_path;
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
		// The board answered - the link is up; the refusal text is the
		// message. Callers toast it exactly once.
		note_conn_ok();
		const message = await response.text();
		throw new Error(message || `An error has occured: ${response.status}`);
	}
	return await response.json();
}

async function start_manual_log_capture() {
	const minutes = parseInt(document.getElementById("manual_log_minutes").value);
	if (isNaN(minutes) || minutes < 1 || minutes > 720) {
		show_toast("Capture length must be 1-720 minutes.", "#7a0f19");
		return;
	}
	try {
		const response = await fetch(ROOT_URL + "api/logs/capture?seconds=" + (minutes * 60), {method: 'POST'});
		if (!response.ok) {
			note_conn_ok();
			show_toast(await response.text() || `An error has occured: ${response.status}`, "#7a0f19");
			return;
		}
		const data = await response.json();
		show_toast("Manual capture started in session " + data.session_id + ".");
	} catch (error) {
		show_toast(normalize_request_error(error).message, "#7a0f19");
		return;
	}
	get_sonar_log_status();
	refresh_log_sessions();
}

async function stop_manual_log_capture() {
	try { await logging_delete("api/logs/capture"); show_toast("Manual capture stopped."); }
	catch (error) { show_toast(error.message, "#7a0f19"); }
	get_sonar_log_status(); refresh_log_sessions();
}

async function delete_selected_log_session() {
	const session = document.getElementById("log_session_select").value;
	if (!session) {
		show_toast("No capture session selected.", "#7a0f19");
		return;
	}
	if (confirm("Delete complete logging session " + session + "?") !== true) return;
	try { await logging_delete("api/logs/session?session=" + encodeURIComponent(session)); show_toast("Session deleted."); }
	catch (error) { show_toast(error.message, "#7a0f19"); }
	get_sonar_log_status(); refresh_log_sessions();
}

async function clear_sonar_log() {
	if (confirm("Clear ALL operational, trip and sonar logs?\nThe FC firmware image is not affected.") !== true) return;
	try {
		const json = await logging_delete("api/logs/all");
		document.getElementById("sonar_persistent_log").value = "All logs cleared.";
		show_toast(json.msg || "All logs cleared.");
	} catch (error) { show_toast(error.message, "#7a0f19"); }
	get_sonar_log_status(); refresh_log_sessions();
}

function get_runtime_firmware_info() {
	get_json("api/update/info").then(json_data => {
		cached_runtime_info = json_data;
		render_about_text();
	}).catch(error => {
		// get_json() already counted this failure - do not count it twice
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
		// get_json() already counted this failure - do not count it twice
		error.message;
		return -1;
	});
	return 0;
}

function format_esp_uptime(uptime_ms) {
	if (!Number.isFinite(uptime_ms) || uptime_ms < 0) {
		return "";
	}
	const total_seconds = Math.floor(uptime_ms / 1000);
	const days = Math.floor(total_seconds / 86400);
	const hours = Math.floor((total_seconds % 86400) / 3600);
	const minutes = Math.floor((total_seconds % 3600) / 60);
	const seconds = total_seconds % 60;
	const clock = [hours, minutes, seconds]
		.map(value => value.toString().padStart(2, "0"))
		.join(":");
	return days > 0 ? days + "d " + clock : clock;
}

/**
 * A request reached the Companion. Clears the failure streak.
 */
function note_conn_ok() {
	conn_fail_count = 0;
	conn_status = 1;
}

/**
 * A request failed to reach the Companion (timeout, abort or network error).
 * Only reports the link as down once the failures are consecutive, so a single
 * dropped packet no longer paints the whole UI red.
 */
function note_conn_fail() {
	conn_fail_count++;
	if (conn_fail_count >= CONN_FAIL_THRESHOLD) {
		conn_status = 0;
	}
}

function update_conn_status() {
	if (conn_status) {
		const uptime = format_esp_uptime(esp_uptime_ms);
		document.getElementById("web_conn_status").innerHTML =
			"<span class=\"dot_green\"></span> connected to Companion" +
			(uptime ? " &middot; uptime " + uptime : "")
	} else {
		document.getElementById("web_conn_status").innerHTML = "<span class=\"dot_red\"></span> disconnected from Companion"
		document.getElementById("current_client_ip").innerHTML = ""
	}
	if (conn_status !== old_conn_status) {
		// connection status changed. Update settings and UI
		get_system_info();
		get_settings();
		get_sonar_log_status();
		refresh_log_sessions();
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
function format_serial_counter(total_bytes, bytes_per_second) {
	if (isNaN(total_bytes)) return "&mdash;";
	if (total_bytes > 1000000) {
		return (total_bytes / 1000000).toFixed(3) + " MB<br>" +
			((bytes_per_second * 8) / 1000).toFixed(2) + " kbit/s";
	}
	if (total_bytes > 1000) {
		return (total_bytes / 1000).toFixed(2) + " kB<br>" +
			((bytes_per_second * 8) / 1000).toFixed(2) + " kbit/s";
	}
	return total_bytes + " bytes<br>" + Math.round(bytes_per_second) + " byte/s";
}

function get_stats() {
	// Called every 500 ms. With a 2500 ms timeout an unanswered request would
	// otherwise stack up behind itself and add load to an already lossy link.
	if (s_stats_in_flight) {
		return 0;
	}
	s_stats_in_flight = true;
	get_json("api/system/stats").then(json_data => {
		note_conn_ok();
		esp_uptime_ms = Number(json_data["esp_uptime_ms"]);
		update_conn_status();
		let d = new Date();
		recv_ser_bytes = parseInt(json_data["read_bytes"]);
		sent_ser_bytes = parseInt(json_data["sent_bytes"]);
		serial_dec_mav_msgs = parseInt(json_data["serial_dec_mav_msgs"]);
		let bytes_per_second = 0;
		let sent_bytes_per_second = 0;
		let current_time = d.getTime();
		if (last_byte_count > 0 && last_timestamp_byte_count > 0 && !isNaN(recv_ser_bytes)) {
			bytes_per_second = (recv_ser_bytes - last_byte_count) / ((current_time - last_timestamp_byte_count) / 1000);
		}
		if (last_sent_byte_count > 0 && last_timestamp_byte_count > 0 && !isNaN(sent_ser_bytes)) {
			sent_bytes_per_second = (sent_ser_bytes - last_sent_byte_count) / ((current_time - last_timestamp_byte_count) / 1000);
		}
		last_timestamp_byte_count = current_time;
		document.getElementById("read_bytes").innerHTML =
			format_serial_counter(recv_ser_bytes, bytes_per_second);
		last_byte_count = recv_ser_bytes;
		document.getElementById("sent_bytes").innerHTML =
			format_serial_counter(sent_ser_bytes, sent_bytes_per_second);
		last_sent_byte_count = sent_ser_bytes;

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
		update_fc_status_chips(json_data);
		change_hardwired_visibility();
		update_hardwired_readouts(json_data);
		update_deeper_readouts(json_data);
		update_deeper_pipeline_diagnostics(json_data);

	}).catch(error => {
		// get_json() already counted this failure - do not count it twice
		error.message;
	}).finally(() => {
		s_stats_in_flight = false;
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

function update_deeper_pipeline_diagnostics(json_data) {
	let elem = document.getElementById("ss_deeper_pipeline");
	if (elem == null) return;
	let n = (key) => { let value = parseInt(json_data[key]); return isNaN(value) ? 0 : value; };
	let requestCount = n("deeper_request_count");
	let depthCount = n("deeper_depth_count");
	let inputLast = n("deeper_last_depth_interval_ms");
	let inputMax = n("deeper_max_depth_interval_ms");
	let publishCount = n("deeper_fc_publish_count");
	let freshPublishCount = n("deeper_fresh_publish_count");
	let skipCount = n("deeper_no_data_skip_count");
	let returnCount = n("deeper_return_count");
	let returnLast = n("deeper_return_last_interval_ms");
	let returnMax = n("deeper_return_max_interval_ms");
	let returnAge = parseInt(json_data["deeper_return_age_ms"]);
	let returnDepth = parseInt(json_data["deeper_return_depth_mm"]);
	let returnSource = n("deeper_return_sysid") + "." + n("deeper_return_compid");

	let input = "Deeper → ESP: " + depthCount + " depth / " + requestCount + " requests" +
		(depthCount > 1 ? ", interval " + inputLast + " ms (max " + inputMax + " ms)" : "");
	let output = "ESP → FC: " + publishCount + " published, " + freshPublishCount + " fresh" +
		(skipCount ? ", " + skipCount + " no-data skips" : "");
	let returned = returnCount === 0 ? "FC → ESP: no DISTANCE_SENSOR returned yet" :
		"FC → ESP: " + returnCount + " returned, interval " + returnLast + " ms (max " + returnMax +
		" ms), " + (returnDepth / 1000).toFixed(2) + " m from " + returnSource +
		(!isNaN(returnAge) && returnAge >= 0 ? " (" + returnAge + " ms ago)" : "");
	elem.textContent = input + " | " + output + " | " + returned;
}

/**
 * Get settings from ESP and display them in the GUI. JSON objects have to match the element ids
 *  returns 0 on success and -1 on failure
 */
function get_settings() {
	get_json("api/settings").then(json_data => {
		console.log("Received settings: " + json_data)
		note_conn_ok();
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
		// get_json() already counted this failure - do not count it twice
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
			note_conn_ok();
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
			note_conn_ok(); // the board answered - the link is up
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
			note_conn_ok();
			show_toast(send_response["msg"])
			get_settings()  // update UI with new settings
		}).catch(error => {
			show_toast(error.message);
		});
	} else {
		console.log("Form was not filled out correctly.")
	}
}
