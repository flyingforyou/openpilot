# Profiles

A profile names a bus, an ECU address pair, the exact UDS requests allowed on it, and CAN signal
layouts. Fields: `vehicle_family, model_year_range, hardware_or_firmware_evidence, bus, bitrate,
id_format, tx_id, rx_id, supported_service_did, evidence_url, verified_on_vehicle, allow_live`.

A live UDS read needs all of: `verified_on_vehicle: true`, `allow_live: true`, a non-empty
`evidence_url` and `hardware_or_firmware_evidence`, `tx_id`/`rx_id`, a `bitrate`, and the exact
request (hex, e.g. `22F190`) listed in `supported_service_did`. `verified_on_vehicle` means a capture
or probe log from *your* car backs the profile -- do not set it from a forum post.

`example_unverified.yaml` ships with everything off. No verified profile ships with the project; make
your own (e.g. `profiles/verified_local.yaml`, untracked) once you have the evidence.
`supported_service_did` is a list of whole request payloads, not bare DIDs.
