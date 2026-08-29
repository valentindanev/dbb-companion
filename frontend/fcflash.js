/* /fcflash page logic.
 *
 * Flashing behaviour is ported unchanged from the HTML string literal that used
 * to live in http_server.c; only the DOM ids it writes into have moved. The
 * parameter dump/load half is new (21-08-2026).
 */

var seq = 0;

function $(i) { return document.getElementById(i); }
function hex(n) { return '0x' + (n >>> 0).toString(16).toUpperCase().padStart(8, '0'); }

function say(m, c) {
    var d = $('console');
    var s = document.createElement('div');
    if (c) s.className = c;
    s.textContent = m;
    d.appendChild(s);
    d.scrollTop = d.scrollHeight;
}
/* pc() = this browser said it. say() with no class = the boat said it. */
function pc(m, c) { say('» ' + m, c || 'pc'); }

/* ---------------------------------------------------------------- flashing */

/* Decode an ArduPilot .apj entirely in the browser: base64 then raw zlib via
 * the platform's DecompressionStream. Keeps zlib and a 1.3 MB decode buffer out
 * of the firmware. A .bin is passed straight through. */
async function decode(f) {
    if (!f.name.toLowerCase().endsWith('.apj')) {
        var b = new Uint8Array(await f.arrayBuffer());
        return { bin: b, board_id: 0, name: f.name };
    }
    var j = JSON.parse(await f.text());
    var raw = Uint8Array.from(atob(j.image), function (c) { return c.charCodeAt(0); });
    if (typeof DecompressionStream === 'undefined')
        throw new Error('This browser cannot decompress .apj. Use a current Chrome/Firefox/Safari.');
    var st = new Blob([raw]).stream().pipeThrough(new DecompressionStream('deflate'));
    var buf = await new Response(st).arrayBuffer();
    return { bin: new Uint8Array(buf), board_id: j.board_id || 0, name: f.name };
}

/* XMLHttpRequest, not fetch: fetch cannot report UPLOAD progress at all, and a
 * 1.2 MB post over the boat's Wi-Fi otherwise looks frozen for many seconds. */
function send(bin, bid, name) {
    return new Promise(function (res, rej) {
        var x = new XMLHttpRequest(), last = -1;
        x.open('POST', '/api/fcflash/upload?board_id=' + bid + '&name=' + encodeURIComponent(name));
        x.upload.onprogress = function (e) {
            if (!e.lengthComputable) return;
            var p = 100 * e.loaded / e.total;
            $('uprog').value = p;
            $('upmsg').textContent = 'Uploading ' + (e.loaded >> 10) + ' / ' + (e.total >> 10) +
                ' KiB (' + p.toFixed(0) + '%)';
            var q = Math.floor(p / 10) * 10;
            if (q > last) { last = q; pc('upload ' + q + '%'); }
        };
        x.onload = function () {
            if (x.status >= 200 && x.status < 300) {
                try { res(JSON.parse(x.responseText)); }
                catch (e) { rej(new Error('bad reply from the boat')); }
            } else rej(new Error(x.responseText || ('HTTP ' + x.status)));
        };
        x.onerror = function () { rej(new Error('network error - did the Wi-Fi drop?')); };
        x.send(bin);
    });
}

function wire_flashing() {
    $('up').onclick = async function () {
        var f = $('file').files[0];
        if (!f) { $('upmsg').textContent = 'Choose a .apj first.'; return; }
        $('up').disabled = true;
        $('uprog').value = 0;
        $('upmsg').textContent = 'Decoding ' + f.name + ' ...';
        pc('decoding ' + f.name);
        try {
            var d = await decode(f);
            if (!d.board_id)
                throw new Error('No board_id in that file - refusing an image we cannot verify against the FC.');
            pc('decoded ' + d.bin.length + ' bytes, board ' + d.board_id);
            var j = await send(d.bin, d.board_id, d.name);
            $('uprog').value = 100;
            $('upmsg').textContent = 'Stored. CRC ' + hex(j.crc) + ' (computed on the boat).';
            /* Deliberately does NOT restate size/board/CRC - the boat logs that
             * itself a moment later, and printing both looked like it happened twice. */
            pc('upload complete, waiting for the boat to verify');
        } catch (e) {
            $('upmsg').textContent = 'Failed: ' + e.message;
            pc('upload failed: ' + e.message, 'err');
        }
        $('up').disabled = false;
    };

    $('del').onclick = async function () {
        if (!confirm('Delete the stored firmware from the boat?')) return;
        await fetch('/api/fcflash/image', { method: 'DELETE' });
    };

    $('flash').onclick = async function () {
        if (!confirm('This ERASES the flight controller and writes new firmware.\n\n' +
            'Do not cut power until it finishes. Continue?')) return;
        $('flash').disabled = true;
        var r = await fetch('/api/fcflash/start', { method: 'POST' });
        if (!r.ok) pc('refused: ' + await r.text(), 'err');
    };
}

async function poll() {
    try {
        var r = await fetch('/api/fcflash/status?since=' + seq);
        var s = await r.json();
        if (s.console) s.console.split('\n').forEach(function (l) { if (l) say(l); });
        seq = s.next_seq;
        $('state').textContent = s.state;
        var bid = s.board_id ? s.board_id : '—';
        $('fbid').textContent = bid;
        $('fbid2').textContent = bid;
        var armed = s.armed ? '<span class="err">ARMED</span>' : '<span class="ok">disarmed</span>';
        $('armed').innerHTML = armed;
        $('armed2').innerHTML = armed;
        $('iname').textContent = s.image_present ? s.image_name : '—';
        $('isize').textContent = s.image_present ? s.image_size + ' bytes' : '—';
        $('ibid').textContent = s.image_present ? s.image_board_id : '—';
        $('icrc').textContent = s.image_present ? hex(s.image_crc) : '—';
        if (s.bytes_total) $('prog').value = 100 * s.bytes_done / s.bytes_total;
        var busy = ['waiting_for_fc', 'rebooting', 'erasing', 'programming', 'verifying'].indexOf(s.state) >= 0;
        var why = '';
        if (s.armed) why = 'Blocked: the vehicle is armed.';
        else if (!s.image_present) why = 'Upload a firmware image first.';
        else if (busy) why = 'Busy — flashing in progress.';
        else if (s.board_id && s.image_present && s.board_id != s.image_board_id)
            why = 'Blocked: FC board ' + s.board_id + ' does not match the image (' + s.image_board_id + ').';
        $('gate').textContent = why;
        $('flash').disabled = !!why;
        if (s.state === 'failed' && s.error) $('gate').innerHTML = '<span class="err">' + s.error + '</span>';
        set_conn(true);
    } catch (e) { set_conn(false); }
    setTimeout(poll, 500);
}

function set_conn(ok) {
    var p = $('web_conn_status');
    if (!p) return;
    p.innerHTML = '<span class="' + (ok ? 'dot_green' : 'dot_red') + '"></span> ' +
        (ok ? 'connected' : 'no link');
}

/* -------------------------------------------------------------- parameters */

function save_blob(text, filename) {
    var b = new Blob([text], { type: 'text/plain' });
    var u = URL.createObjectURL(b);
    var a = document.createElement('a');
    a.href = u;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    setTimeout(function () { URL.revokeObjectURL(u); }, 4000);
}

function stamp() {
    var d = new Date(), p = function (n) { return String(n).padStart(2, '0'); };
    return d.getFullYear() + p(d.getMonth() + 1) + p(d.getDate()) + '-' +
        p(d.getHours()) + p(d.getMinutes());
}

function wire_params() {
    $('pdump').onclick = async function () {
        $('pdump').disabled = true;
        $('pdprog').value = 0;
        $('pdumpmsg').textContent = 'Asking the flight controller for every parameter…';
        pc('parameter dump requested');
        try {
            var r = await fetch('/api/fcparams/dump', { method: 'POST' });
            if (!r.ok) throw new Error(await r.text() || ('HTTP ' + r.status));
            /* The FC streams ~1400 parameters over a slow link, so the boat
             * collects them in the background and we poll for completion. */
            var done = false, text = '';
            while (!done) {
                await new Promise(function (r2) { setTimeout(r2, 700); });
                var s = await (await fetch('/api/fcparams/status')).json();
                if (s.total) $('pdprog').value = 100 * s.received / s.total;
                $('pdumpmsg').textContent = 'Received ' + s.received +
                    (s.total ? ' of ' + s.total : '') + '…';
                if (s.state === 'failed') throw new Error(s.error || 'dump failed');
                if (s.state === 'complete') done = true;
            }
            $('pdprog').value = 100;
            var n = text.split('\n').filter(function (l) { return l.trim() && l[0] !== '#'; }).length;
            save_blob(text, 'dbb-fc-' + stamp() + '.param');
            $('pdumpmsg').textContent = 'Saved ' + n + ' parameters.';
            pc('parameter dump complete: ' + n + ' parameters', 'ok');
        } catch (e) {
            $('pdumpmsg').textContent = 'Failed: ' + e.message;
            pc('parameter dump failed: ' + e.message, 'err');
        }
        $('pdump').disabled = false;
    };

    /* Parse client-side so the count can be shown BEFORE anything is written. */
    var parsed = null;
    $('pfile').onchange = async function () {
        parsed = null;
        $('pload').disabled = true;
        var f = $('pfile').files[0];
        if (!f) { $('ppreview').textContent = ''; return; }
        var text = await f.text();
        var out = [], bad = 0;
        text.split(/\r?\n/).forEach(function (line) {
            var t = line.trim();
            if (!t || t[0] === '#') return;
            var m = t.split(/[,\s]+/);
            if (m.length < 2) { bad++; return; }
            var v = parseFloat(m[1]);
            if (!m[0] || !isFinite(v)) { bad++; return; }
            out.push({ id: m[0].toUpperCase().slice(0, 16), value: v });
        });
        if (!out.length) {
            $('ppreview').innerHTML = '<span class="err">No usable parameters in that file.</span>';
            return;
        }
        parsed = out;
        $('ppreview').innerHTML = '<b>' + out.length + '</b> parameters parsed from <code>' +
            f.name + '</code>' + (bad ? ', <span class="warn">' + bad + ' line(s) skipped</span>' : '') +
            '. Nothing has been written yet.';
        $('pload').disabled = false;
    };

    $('pload').onclick = async function () {
        if (!parsed) return;
        if (!confirm('Write ' + parsed.length + ' parameters to the flight controller?\n\n' +
            'This changes how the boat flies. Make sure you have dumped the current ' +
            'parameters first. Continue?')) return;
        $('pload').disabled = true;
        $('plprog').value = 0;
        $('presult').style.display = 'none';
        $('presult').textContent = '';
        $('ploadmsg').textContent = 'Writing…';
        pc('parameter load started: ' + parsed.length + ' parameters');
        try {
            var r = await fetch('/api/fcparams/load', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ params: parsed })
            });
            if (!r.ok) throw new Error(await r.text() || ('HTTP ' + r.status));
            await watch_load();
            await refresh_stored();
        } catch (e) {
            $('ploadmsg').textContent = 'Failed: ' + e.message;
            pc('parameter load failed: ' + e.message, 'err');
        }
        $('pload').disabled = false;
    };
}



/* ---- the backup stored on the boat ------------------------------------- */

async function refresh_stored() {
    var info = { present: false, count: 0, bytes: 0 };
    try {
        info = await (await fetch('/api/fcparams/stored')).json();
    } catch (e) { /* leave the defaults; the kv row shows the dash */ }
    $('pstored').textContent = info.present ? 'present' : 'none';
    $('pstoredn').textContent = info.present ? info.count : '—';
    $('pstoredb').textContent = info.present ? info.bytes + ' bytes' : '—';
    $('prestore').disabled = !info.present;
    $('pdownload').disabled = !info.present;
    $('pdelete').disabled = !info.present;
    return info;
}

function wire_stored() {
    $('pdownload').onclick = async function () {
        try {
            var text = await (await fetch('/api/fcparams/file')).text();
            save_blob(text, 'dbb-fc-' + stamp() + '.param');
            pc('backup downloaded to this device');
        } catch (e) { pc('download failed: ' + e.message, 'err'); }
    };

    $('pdelete').onclick = async function () {
        if (!confirm('Delete the parameter backup stored on the boat?\n\n' +
            'This is your way back from a bad reflash.')) return;
        await fetch('/api/fcparams/stored', { method: 'DELETE' });
        await refresh_stored();
        pc('stored backup deleted', 'warn');
    };

    $('prestore').onclick = async function () {
        var info = await refresh_stored();
        if (!info.present) return;
        if (!confirm('Write the stored backup (' + info.count + ' parameters) to the ' +
            'flight controller?\n\nUse this after reflashing the FC. Keep it disarmed.')) return;
        $('prestore').disabled = true;
        $('plprog').value = 0;
        $('ploadmsg').textContent = 'Restoring…';
        pc('restore from the stored backup started: ' + info.count + ' parameters');
        try {
            var r = await fetch('/api/fcparams/restore', { method: 'POST' });
            if (!r.ok) throw new Error(await r.text() || ('HTTP ' + r.status));
            await watch_load();
        } catch (e) {
            $('ploadmsg').textContent = 'Failed: ' + e.message;
            pc('restore failed: ' + e.message, 'err');
        }
        await refresh_stored();
    };
}

/* Shared by "restore from the boat" and "load from this device". */
async function watch_load() {
    var done = false, res = null;
    while (!done) {
        await new Promise(function (r2) { setTimeout(r2, 700); });
        var s = await (await fetch('/api/fcparams/status')).json();
        if (s.total) $('plprog').value = 100 * s.written / s.total;
        $('ploadmsg').textContent = 'Written ' + s.written + ' of ' + s.total + '…';
        if (s.state === 'failed') throw new Error(s.error || 'load failed');
        if (s.state === 'complete') { done = true; res = s; }
    }
    $('plprog').value = 100;
    var msg = 'Wrote ' + res.written + ' of ' + res.total + ' parameters.';
    if (res.failed && res.failed.length) {
        msg += ' ' + res.failed.length + ' did NOT take.';
        $('presult').style.display = '';
        $('presult').textContent = 'These parameters were not accepted by the FC:\n' +
            res.failed.join('\n');
        pc(msg, 'warn');
    } else {
        pc(msg, 'ok');
    }
    $('ploadmsg').textContent = msg;
    return res;
}

function refresh_all() { seq = 0; refresh_stored(); }


/* ---------------------------------------------------------------- auto-tune */

var tseq = 0;             /* console ring cursor */
var tmodal_shown = false; /* so the result dialog opens once per run */
var tune_next_ms = 2500;  /* poll fast only while a tune is actually running */

function tsay(m) {
    var c = $('tconsole');
    if (!c) return;
    var atBottom = c.scrollHeight - c.scrollTop - c.clientHeight < 24;
    var d = document.createElement('div');
    d.textContent = m;
    c.appendChild(d);
    while (c.childNodes.length > 300) c.removeChild(c.firstChild);
    if (atBottom) c.scrollTop = c.scrollHeight;
}

function tnum(v, dp, unit, ok) {
    var s = (v === undefined || v === null) ? '—' : v.toFixed(dp) + (unit || '');
    return '<span class="' + (ok ? 'ok' : 'warn') + '">' + s + '</span>';
}

function tune_rows(res, applied) {
    var b = $('tmodal-rows');
    b.innerHTML = '';
    res.forEach(function (r) {
        var tr = document.createElement('tr');
        tr.innerHTML = '<td>' + r.id + '</td>' +
            '<td class="n">' + r.before.toFixed(4) + '</td>' +
            '<td class="n new">' + r.after.toFixed(4) + (applied && r.applied ? ' ✓' : '') + '</td>';
        b.appendChild(tr);
    });
}

function tune_open_modal(s) {
    tune_rows(s.results, false);
    $('tmodal-h').textContent = 'Tune complete — nothing written yet';
    $('tmodal-sub').textContent =
        'The boat measured itself and worked out these gains. Below are the values ' +
        'currently on the flight controller and what they would become.';
    $('tmodal-note').innerHTML =
        '<b>Apply</b> writes them to the flight controller and verifies each one by ' +
        'reading it back. <b>Discard</b> changes nothing.';
    $('tapply').style.display = '';
    $('tdiscard').textContent = 'Discard';
    $('tmodal').classList.add('open');
    tmodal_shown = true;
}

function tune_close_modal() { $('tmodal').classList.remove('open'); }

function wire_tune() {
    $('tstart').onclick = async function () {
        var axes = ($('tax_str').checked ? 1 : 0) | ($('tax_spd').checked ? 2 : 0);
        if (!axes) { $('tmsg').textContent = 'Pick at least one axis.'; return; }
        $('tconsole').innerHTML = '';
        tseq = 0;
        tmodal_shown = false;
        $('tmsg').textContent = '';
        var r = await fetch('/api/fctune/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ axes: axes })
        });
        if (!r.ok) $('tmsg').textContent = 'refused: ' + await r.text();
    };

    $('tabort').onclick = async function () {
        await fetch('/api/fctune/abort', { method: 'POST' });
        tune_close_modal();
    };

    $('tapply').onclick = async function () {
        $('tapply').disabled = true;
        var r = await fetch('/api/fctune/apply', { method: 'POST' });
        if (!r.ok) {
            $('tmodal-note').innerHTML = '<span class="err">refused: ' +
                (await r.text()) + '</span>';
            $('tapply').disabled = false;
            return;
        }
        $('tmodal-h').textContent = 'Writing to the flight controller…';
        $('tmodal-note').textContent = 'Each parameter is verified by reading it back.';
    };

    $('tdiscard').onclick = async function () {
        if ($('tapply').style.display === 'none') { tune_close_modal(); return; }
        await fetch('/api/fctune/abort', { method: 'POST' });
        tune_close_modal();
    };
}

async function tune_poll() {
    try {
        var r = await fetch('/api/fctune/status?since=' + tseq);
        var s = await r.json();
        if (s.console) s.console.split('\n').forEach(function (l) { if (l) tsay(l); });
        tseq = s.seq;

        $('tstate').textContent = s.state + (s.error ? ' — ' + s.error : '');
        $('tarmed').innerHTML = s.armed
            ? '<span class="ok">armed</span>'
            : '<span class="warn">disarmed</span>';

        if (s.live_valid) {
            $('tlsteer').innerHTML = tnum(s.live_steering, 2, '', Math.abs(s.live_steering) >= 0.10);
            $('tlrate').innerHTML = tnum(s.live_turnrate, 0, ' °/s', Math.abs(s.live_turnrate) >= 10);
            $('tlthr').innerHTML = tnum(s.live_throttle * 100, 0, ' %', s.live_throttle >= 0.20);
            $('tlspd').innerHTML = tnum(s.live_speed, 2, ' m/s', s.live_speed > 0.5);
        } else {
            $('tlsteer').textContent = $('tlrate').textContent = '—';
            $('tlthr').textContent = $('tlspd').textContent = '—';
        }

        $('tsprog').value = s.steer_pct;
        $('tvprog').value = s.speed_pct;
        $('tsteerlbl').textContent = s.steer_samples + ' samples, ' +
            Math.round(s.rotation_deg) + '° of 360°';
        $('tspeedlbl').textContent = s.speed_samples + ' samples';

        var running = ['baseline', 'steering', 'speed', 'writing'].indexOf(s.state) >= 0;
        $('tstart').disabled = running || s.state === 'ready';
        $('tabort').disabled = !running && s.state !== 'ready';

        if (s.state === 'ready' && !tmodal_shown && s.results.length) {
            tune_open_modal(s);
        }
        if (s.state === 'complete' && s.results.length) {
            tune_rows(s.results, true);
            $('tmodal-h').textContent = s.error ? 'Applied with problems' : 'Applied';
            $('tmodal-sub').textContent = s.error
                ? s.error
                : 'Every value was written and read back from the flight controller.';
            $('tmodal-note').textContent =
                'These are now live on the FC. Dump parameters if you want a backup ' +
                'that includes them.';
            $('tapply').style.display = 'none';
            $('tdiscard').textContent = 'Close';
            $('tmodal').classList.add('open');
        }
        if (s.state === 'failed') {
            $('tapply').disabled = false;
        }
        /* Back off hard when nothing is running. This page already polls
         * /api/fcflash/status twice a second; TOPOLOGY records sustained ESP
         * HTTP polling at a few req/s wedging the HTTP server and blocking a
         * GCS, so the idle cost of this card has to stay near zero. */
        tune_next_ms = running || s.state === 'ready' ? 500 : 2500;
    } catch (e) { tune_next_ms = 2500; }
    setTimeout(tune_poll, tune_next_ms);
}

window.addEventListener('DOMContentLoaded', function () {
    wire_flashing();
    wire_params();
    wire_stored();
    wire_tune();
    refresh_stored();
    poll();
    tune_poll();
});
