# cf-sound-localization

This project uses nRF54L15-DKs and ICS-43434 I2S microphones to collect synced audio data, which can then be processed with TDoA methods to localize sound sources.

- `conn_time_sync/` includes the firmware for the nRF54L15-DKs.
- `scripts/` includes a Jupyter notebook with sound localization implementations and example audio data.

This application is heavily based on the [Bluetooth: Connection time synchronization](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/samples/bluetooth/conn_time_sync/README.html) sample from Nordic.
