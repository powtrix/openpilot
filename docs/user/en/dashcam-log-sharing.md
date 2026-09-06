# Sending Dashcam Logs for Analysis

[한국어](../ko/dashcam-log-sharing.md)

When you ask a Carrot support specialist to analyze abnormal behavior, use `Logs > Dashcam` in Carrot Web to find and upload the affected time range. A dashcam upload can provide vehicle-state and control-decision data in addition to the visible road video.

> [!WARNING]
> Operate Carrot Web only after parking safely. While driving, do not search for or select logs; remember the occurrence time and symptom instead.

## Automatic community sharing versus manual log upload

`System > Record & Power > DK Automatic External Logs & Diagnostics` is the master consent for comma Athena and other automatic transfers to outside third parties; `Carrot Community Data Sharing` is the subordinate consent for Carrot community services. Both are off by default, and both must be enabled before Carrot community servers or bundled Discord webhooks can receive device/network status, setting values, automatic onroad/exception tmux diagnostics, and support metadata without per-file confirmation. Popular-setting downloads and CWP address registration also require both. Turning either one off blocks new community requests, stops all automatic tmux capture/retries, and discards pending automatic exception transfers.

Turning both sharing settings off does not change a manual upload itself after it is explicitly started from selected segments in `Logs > Dashcam`, a user-configured NAS or Discord URL, or the separate KA4 automatic-validation consent below. However, the default `adot.synology.me` DK receiver is restricted to automatic KA4 validation and rejects manual dashcam/tmux uploads with HTTP 403; manual upload requires a separately authenticated private receiver. The post-upload report to the bundled default Discord is also blocked, so copy the completion result and send it manually. In particular, if `CarrotValidationAutoUpload` is enabled, its full-rlog upload to the fixed private-NAS receiver can continue while both sharing settings are off. None of the three consent values is included in backups, profiles, or QR transfer, so a new installation or another device cannot inherit it automatically.

<a id="automatic-validation-upload"></a>
## Automatic KA4 validation upload (experimental)

`System > Record & Power > KA4 Automatic Validation Log Upload (Experimental)` is a bounded collector that removes the need to find or send each log manually. It is off by default. Enabling it once while safely parked is explicit consent for a campaign of up to seven days, including every automatic post-drive upload during that campaign; there is no per-log confirmation. It never changes vehicle-control behavior or `PathOffset`; `Ka4StockSccStandstillRearm` is internal metadata that records automatic applicability rather than a separate user setting.

The collector arms only when all of this vehicle topology is confirmed:

- The owner's allowlisted DK device only; its raw identifier is not published in the branch
- Fourth-generation Kia Carnival KA4 (the current vehicle-validation target is model year 2023)
- CAN FD stock radar SCC and PCM cruise
- No openpilot longitudinal control and no camera SCC

During a drive it distinguishes an experimental RES frame request being queued while automatic behavior applies from a qualified stop where no controller request occurred, for failure analysis. The OFF-state IDs (`standstill_off` and `standstill_off_physical_res`) are restore-only compatibility classifications: an existing capture from an older version can still upload, but a new campaign neither creates nor requires them. A physical RES press remains visible in the retained rlog rather than creating a new legacy OFF event. The collector also distinguishes lane-mode `PathOffset=0` from `PathOffset=10` (10 cm right) while `AdjustLaneOffset=0`. It selects sustained acceleration of at least `0.7 m/s²` for `0.5 s` while stock SCC is active, the driver is not pressing a pedal, and the car is closing on a nearby lead. `carState.ka4StockSccKeepaliveRequestCount` records only controller frames appended to the outgoing CAN list, together with the qualification epoch. It does not prove Panda transmission or stock-SCC ECU reception or acceptance, and the ON event therefore means “controller request observed,” not “vehicle behavior changed.”

When a condition is detected, the triggering segment and up to two contiguous preceding segments receive a dedicated temporary retention marker. For acceleration while closing on a lead, one following segment is included when it finishes so the subsequent braking response is available; the total remains capped at three. That marker is separate from driver-created log bookmarks, so completion or consent withdrawal never clears a driver bookmark. Upload begins only after the drive has ended, the device is stopped and off-road, and Wi-Fi is connected. No file selection or upload button is required. An in-progress upload is canceled when consent is withdrawn, driving resumes, Wi-Fi is lost, or the campaign expires.

Each event capture sends **at most three full rlogs only**. A campaign can collect each of seven conditions up to twice, for **at most 14 captures / 42 full rlogs**. At most 5 captures / 750 MiB can wait locally at once, but this is a concurrent pending-data cap, not a cap on cumulative campaign uploads or retry traffic. The collector does not separately send `qcamera`, driver-camera, tmux data, or a Discord notification. A full rlog can nevertheless contain precise location, vehicle CAN and control state, device identifiers, branch, commit and working-tree modification status, Params captured at route start, and low-resolution road thumbnails sampled at roughly one-minute intervals. It does not photograph the Kia cluster or its exact alert text, so the text itself cannot be proven from an rlog alone.

For upload, the device signs a purpose-specific value bound only to that request's one-time challenge with its existing registration key. The receiver verifies the signature locally against a privately pre-enrolled public-key fingerprint; it neither receives a general comma API bearer nor contacts the official comma device API. The user does not enter a token or password. Automatic validation uploads go only to the trusted HTTPS receiver built into the branch, or to an immutable receiver fixed by the system administrator at deployment time. The ordinary Carrot Web upload destination cannot redirect them. The receiver recomputes every file's size and SHA-256, and the device removes a local queue entry only after the capture ID, verified device ID, complete file list, and final manifest hash all match the completion receipt. If the receiver does not support this authenticated protocol or identity verification is temporarily unavailable, the logs remain retained locally and retry automatically.

The queue survives a reboot. Failures retry after approximately 30 seconds, 2 minutes, 10 minutes, 1 hour, and 6 hours, with small timing jitter. Collection is limited to two captures per condition, five concurrently pending captures overall, and about 750 MiB pending. The receiver has a 1 GiB per-device daily limit, so retained local logs may retry automatically the next day after the limit is reached. The setting row shows only a sanitized state, pending count, expiry, and last-upload time; it never shows a route, receiver URL, device ID, capture ID, or raw error. The setting turns itself off after all three required conditions—a standstill controller request, `PathOffset=10` lane operation, and stock-SCC acceleration while closing on a lead—upload, or after seven days. The no-request stop and `PathOffset=0` lane captures are optional diagnostics; the two legacy OFF IDs never count toward completion. Turning the setting off manually discards pending entries and releases retention markers created by this feature. Consent is excluded from settings backups, restores, and QR transfer; if the saved campaign state is unavailable after a reinstall, explicitly toggle the setting off and on again while parked.

Carrot Web has no user login and treats the local network as its trust boundary. Use it only on a private WPA2/WPA3 tether or hotspot with a strong unique password, not on public Wi-Fi. Automatic uploads over phone tethering can use mobile data; successive captures and retries can make total data use exceed the 750 MiB concurrent local queue limit. Disabling the setting does not delete data already stored on the server; ask the server administrator if deletion is required.

## Record these details first

Note as much of the following as possible:

- Date and time, preferably accurate to the minute
- Observed symptom, such as unexpected deceleration, failure to start, lane departure, or an alert sound
- Conditions, including road type, approximate speed, lead vehicle, curve, merge, congestion, or signal state
- Whether it repeated and any driver intervention with the brake, accelerator, steering wheel, or buttons
- Vehicle, current branch, and commit, available from `Tools > Info`

“At about 14:32, the vehicle unexpectedly decelerated from about 80 km/h as a car merged from the right, and I disengaged with the brake” is much more useful than “It behaved strangely.”

## Fastest method: send a symptom that just occurred

1. Park safely and wait for the drive to finish.
2. Open `http://device-IP:7000` in a browser.
3. Open `Logs > Dashcam`.
4. From the top-right Logs menu, select `Upload recent 5 logs`.
5. Review the file count and upload size, then confirm.
6. When it finishes, verify that the successful count equals the total and select `Copy`.
7. Send the copied result with the symptom details to the designated support specialist.

Recent-log upload selects the newest completed segments regardless of the visible sort order. `Recent 5` is recommended for a typical one-time event because it usually includes useful context before and after it.

| Selection | Appropriate use |
|---|---|
| Recent 2 | The time is certain and upload size must be minimized |
| Recent 5 | A recent one-time symptom or normal analysis request |
| Recent 10 | The exact segment is uncertain or the symptom repeated over a longer period |

## Find and send an exact range

Dashcam logs are shown as drive cards containing segments of about one minute each.

1. Expand the drive card containing the occurrence date and time.
2. Use the displayed segment times to locate the event.
3. Select a segment to play its video, or use `Replay` from its menu to inspect it with recorded driving data.
4. Check the affected segment.
5. When possible, also select the segment immediately before and after it. Analysis often depends on what happened around the symptom.
6. Use `Upload selected`, Select all, or range selection for multiple segments. Range input accepts forms such as `1`, `1-3`, or `1, 3-5`.
7. In the confirmation dialog, review the file count, total size, and mobile-data warning.
8. Start the upload and keep the device online until it completes. You can cancel from the progress dialog if necessary.

A segment that is still recording or has not been finalized cannot be uploaded. If a newly completed segment is not visible, wait briefly. While the Logs page is active, it silently checks for updates about every 10 seconds.

## Verify the result and request analysis

If the completion screen shows the same `uploaded/total` number, all selected files were uploaded. If an entry shows `FAILED` or the successful count is lower, retry the failed segment.

The result generated by `Copy` can include device, branch, commit, and per-segment upload information. Each successful segment is shown as a public Synology viewer link. Opening it provides web playback, public files, and ready-to-copy Cabana, PlotJuggler, and JotPluggler commands. Consecutively numbered segments from the same route also receive one combined link that opens the full range.

When the upload job finishes, the device sends the same result report directly to the configured Discord webhook. If the user has not supplied a custom webhook and the bundled default Discord is used, both `DkThirdPartyDataSharing=1` and `CarrotCommunityDataSharing=1` are required. Disabled consent or a Discord notification failure does not undo an already completed log upload; post the completion screen's `Copy` result to the channel manually.

Public viewer links do not require a login. Anyone who receives a link can open or forward it. Confirm the selected segments and the Discord channel where the report will be posted, then add the following details when requesting analysis:

```text
Vehicle:
Occurrence date/time:
Symptom:
Speed and road conditions:
Lead or surrounding vehicle conditions:
Driver intervention:
Number of reproductions:
Upload result: (paste the text copied from Carrot Web)
Screen recording: yes / no
```

Only one upload job can run at a time. If the browser is briefly closed and reopened while the same device-side job is still running, Carrot Web may restore its progress or result. A job ID saved by the browser is verified against the device before it is used, so an already-finished or missing job does not block a new upload by itself. A device-side upload with no activity for 30 minutes is marked failed and released so that it cannot block later uploads.

## Include a screen recording when useful

A Carrot Web screen recording helps explain what the driver saw, including HUD, alerts, and visible UI changes.

1. Before driving, select `Record` on the Drive page.
2. Confirm that the `REC` indicator appears.
3. After parking safely, select `Record` again to stop.
4. Open `Logs > Screen Record`, play the file to confirm the symptom is visible, and download it separately if the specialist requests it.

A screen recording alone may not contain enough data to determine the control cause. For abnormal driving behavior, upload the dashcam logs from the same time first.

## Troubleshooting

| Symptom | What to check |
|---|---|
| The latest drive is missing | Confirm that the drive has completely ended, wait briefly, and check the list again. |
| Recording or incomplete-segment error | Wait for that segment to be finalized, then select it again. |
| Another upload is already running | Carrot Web rechecks the existing device-side job and restores its progress. If it is genuinely running, wait for it to finish or cancel it. A job with no activity for 30 minutes is released automatically. |
| Upload stalls or fails | Check the device's internet connection and retry only the failed segment. |
| Only some files succeeded | Read the completion result and resend the segment containing the failed item. |
| The required log is gone | Check whether `Tools > delete all logs` or storage cleanup was used. Deleted logs cannot be restored or uploaded from Carrot Web. |

Use the segment menu's `qcamera`, `rlog`, or `qlog` download only when a specialist asks for a particular original file. For a normal analysis request, use `Upload Logs` or `Upload selected`.

## Privacy and sharing

Uploaded data may include road video, location and vehicle-state logs, device identifiers, vehicle name, branch, and commit information. Recheck the selected time and segments before uploading because a public viewer link remains available to anyone while the files remain on the server. Driver-facing `dcamera` files are excluded from the public viewer and analysis API.

## Related guides

- [Carrot Web User Guide](carrot-web.md)
- [Understanding Settings](settings.md)
