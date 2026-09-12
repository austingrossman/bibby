# bibby — TODO

Work that is understood but not done. Each entry should carry enough context to
be picked up cold: what, why, and what was already ruled in or out.

---

## HTTPS for the web interface

**Status:** not started. The web interface (README §2.9, §8) serves plain HTTP,
so its HTTP Basic password crosses the LAN in the clear — it is base64, not
encryption. Everything else about the feature is done.

**Why it matters.** The password is the only authorization on a browser that
can drive the panel. Home Wi-Fi already encrypts the air link (WPA2/WPA3), so
the realistic exposure is another device on the LAN or the wired segment, not a
passer-by. Secondary but real wins: browsers stop flagging the page as insecure,
and password managers will offer to save the credential. The biggest gain is
that Basic auth stops being the weak link — over TLS it is a reasonable scheme.

**Today's workaround**, already documented in README §8: `web.bind = 127.0.0.1`
plus `ssh -L 8080:localhost:8080 ramp`. Strong crypto, zero new code, but
awkward from a phone.

### The code is the easy part (~200 lines)

All socket I/O goes through three call sites:

| Location | Call |
|---|---|
| `src/web/http.c` `http_read_request()` | `recv()` for headers |
| `src/web/http.c` `http_read_request()` | `recv()` for the body |
| `src/web/http.c` `http_send_all()` | `send()` |

(The `read()` in `http_respond_file()` is the log file, not the socket — it
already goes out through `http_send_all`.)

So the shape is: add an `SSL *` to `HttpReq`, route those three through a
dispatch on whether TLS is active, create the `SSL_CTX` in `web_start()`, and
do `SSL_new` / `SSL_accept` / `SSL_shutdown` / `SSL_free` per connection in
`conn_main()`. New config keys alongside the existing `[web]` block:
`tls_cert`, `tls_key` — absent means plain HTTP, matching how an absent
password already disables the server entirely.

Notes for whoever does it:

- OpenSSL 3.5.7 is already on the Pi; `apt install libssl-dev` for the headers,
  then link `OpenSSL::SSL` in `CMakeLists.txt`.
- Since OpenSSL 1.1.0 the library is thread-safe for distinct `SSL` objects, so
  the thread-per-connection design needs no locking callbacks.
- The one subtle part: the connection sockets carry `SO_RCVTIMEO` (30 s), and a
  socket timeout surfaces from OpenSSL as `SSL_ERROR_WANT_READ`, not an error.
  Handshakes need their own timeout so a client that opens a socket and says
  nothing cannot hold a thread.
- Performance is a non-issue. The Pi 5 has ARM crypto extensions and measures
  2.28 GB/s AES-128-GCM (`openssl speed -evp aes-128-gcm`); the screen mirror
  at 5 fps is 240 kB/s.

### Certificates are the real work

This is the decision to make first — the code follows from it.

| Approach | Effort | Result |
|---|---|---|
| Self-signed | ~10 min | Warning screen on every device, every browser |
| **Local CA** (mkcert, or plain openssl) | ~1 h + per-device trust install | Clean padlock, 10-year cert, no renewals |
| Let's Encrypt via DNS-01 | ~half day | No warnings anywhere, but needs a domain you own, outbound internet, and 90-day renewal automation on an appliance |
| Reverse proxy (Caddy/nginx/stunnel) | ~30 min config, no C | Sidesteps all of the above, but adds a daemon and a failure mode, and gives up the single-binary property |

The Pi runs `avahi-daemon` and answers to **`ramp-controller.local`**, so issue
the certificate against that name rather than `192.168.1.40` — it then survives
a DHCP change.

**Recommendation:** in-process OpenSSL with a local CA, keeping plain HTTP
working when no cert is configured. Budget about a day including testing. The
unavoidable annoyance is installing the CA root on each device: iOS needs a
configuration profile *and* a separate trust toggle buried in Settings, and
Firefox on Android keeps its own trust store separate from the system one.

**When it lands,** update README §2.9 and §8 (which currently tell the operator
plainly that traffic is unencrypted), the `[web]` comment block in `bibby.ini`,
and the safety note in `CLAUDE.md`.

---

## Approach mode: full power to setpoint, then cut to feedforward

**Status:** not started. Proposed after the 2026-09-07 ramp test
(`logs/2026/09/07/18-59-53.csv`), alongside the two changes that did land
from that analysis: integral separation (`pid.i_band_c`) and the retune to
τc = 30 s. Those two fix the overshoot; this item is about getting to
setpoint faster than a tapering P term ever can.

**The idea, in full.** When the error exceeds a configured threshold, command
full power (or the grain-in power cap while Grain In is on), hold the
integrator at zero, and cut to feedforward-only when the predicted coast
reaches the setpoint, then hand control to the PI. Above `max_power / kp` of
error the PI already commands full power, so the real addition is the *cut*
rule: switch to holding feedforward when the temperature will coast onto the
setpoint, instead of letting the P term taper the last couple of degrees.
The coast is best predicted from the measured slope times the dead time
(`dT/dt × L`, with the slope taken over roughly the last 10 s of the filtered
temperature) rather than from the batch-size estimate, because the slope is
available immediately, includes heat loss, and does not depend on the
estimator having converged. The batch-size estimate (`m_est_l`) is still
worth using for a time-to-setpoint readout on the UI and as a cross-check on
the slope. With `kp = 5633 W/°C` the P term tapers from full power below
1.9 °C of error; at the measured 2 °C/min full-power ramp that taper costs
about a minute per step, which is what this mode would save.

**Findings to design against** (from the 2026-09-07 heat/cool log,
`logs/2026/09/07/15-41-11.csv`):

- Dead time is ~9.5 s: the reading starts rising 9 to 12 s after power-on.
- After a hard cut from ~7.7 kW at 1.9 °C/min the reading coasted up
  +0.2 °C, peaking 12 to 15 s after the cut. Slope × L predicts +0.3 °C, so
  the prediction is conservative by about a third.
- After that peak the reading *sagged* 0.3 °C over the next two minutes as
  the stratified layer around the probe collapsed once the element plume
  stopped (the identification script calls this the mixing offset). So the
  cut point should be about 0.2 °C below setpoint, and the PI must then
  recover a 0.3 °C sag promptly, which the τc = 30 s gains can do.

**Shape of the implementation.** A small state machine in
`src/sampler_thread.c` ahead of `pid_update()`: `APPROACH` entered when
`error > approach.engage_c` in auto mode and not faulted; while in it,
`p_demand = out_max_w`, `pid_reset()` each pass so the integrator stays at
zero, and a slope estimator over the last N samples of `temp_filt`; exit to
`TRACK` (plain PI) when `temp_filt + slope × approach.coast_s >= setpoint -
approach.margin_c`, or if the operator drops the setpoint below the current
temperature, or on any fault (the existing interlock already forces manual).
New `[approach]` keys in `bibby.ini`: `enable`, `engage_c`, `coast_s`
(default L from the identification), `margin_c`, all documented in README §7
and §9. Log the mode (a new CSV column or a value in `manual`-style flags)
so the cut can be checked in `tools/plot_logs.py`. Unit-test the state
machine with a synthetic ramp. Safety: this mode commands nothing the PI
cannot already command (full power is the existing clamp), and every
existing interlock runs ahead of it, so it adds no new failure mode.


---

**Grain In profile** Do we need it
I read the following in the README:
"Grain in makes the plant slower (more mass, worse mixing), which errs on the safe side for a fixed gain set but still wants its own [grain] gains."
First of all, the plant from a control standpoint is actually a hybrid. Most of the flow happens in the shallow area below the false bottom, and i expect the mass estimate to go low and the temp to more quickly servo. But, a fraction of the water is recirculated from the top and pushed down into the temp controlled area. I expect the integrator to do the work here tracking the needed heat to keep the new flowing wort in at good temp control until, eventually, the grain is actually at the right temp, at which point the integrator will need to lower.
However, i wonder if the new profile is even needed at all considering the mass estimator is working correctly...however, the mass estimator needs a step command in temp to work reasonably. Perhaps we need a separate pid tuning with a lower mass (just above the grain bed of water to emulate).
