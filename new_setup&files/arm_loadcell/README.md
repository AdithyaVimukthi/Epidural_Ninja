# Load cell experiment recorder

PySide6 desktop GUI for a Seeed Studio XIAO ESP32S3, Adafruit HX711,
and 1 kg load cell. Records uncalibrated signed ADC counts, not grams.

## Board setup

- Install `esp32 by Espressif Systems` in Arduino IDE Boards Manager.
- Select `XIAO_ESP32S3`; enable `USB CDC On Boot`.
- Install `Adafruit HX711` in Library Manager.
- Wire HX711 DATA to D7, SCK to D8, VIN to 3V3, and GND to GND.
- Connect the load cell to E+, E-, A+, A- according to its datasheet.
- Upload `loadcell_raw/loadcell_raw.ino`.
- Close Arduino Serial Monitor before connecting the Python app.

## Install and launch (macOS/Linux)

Use Python 3.10 or newer. From this project directory:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python recorder.py
```

On Windows, create the environment with `py -m venv .venv`, then activate
with `.venv\Scripts\activate` in Command Prompt.

## Run an experiment

1. Select the USB port, click Connect, and allow a few seconds for board startup.
2. Enter an experiment name and choose the save folder.
3. Click Start experiment. Recording starts after the board acknowledges it.
4. Click Stop and plot. The app drains samples up to the stop acknowledgement,
   closes the CSV, and plots raw counts against elapsed seconds.
5. Use the graph toolbar to zoom/pan. Save graph PNG exports the graph.
6. Start another experiment for a new uniquely named CSV, or Open saved CSV
   to plot an earlier experiment.

Enable Demo without connecting a board to test the full workflow with simulated
10 Hz readings. Demo filenames contain `_DEMO` so they can be identified.

## Data and timing

CSV columns are `elapsed_ms,raw_value`. Time comes from the board, relative to
its receipt of the start command. Samples are written and flushed individually;
a disconnection preserves samples already received. A force-quit may lose bytes
still in USB buffers. A normal stop waits up to five seconds for acknowledgement.
If interrupted, the graph displays the partial experiment.

The HX711 hardware determines sample rate (typically 10 or 80 samples/second).
The app does not average, tare, resample, or convert readings to mass. For grams,
calibrate using a known weight in a later step. Secure the load cell consistently
between experiments and do not exceed its rated load.

Firmware commands are newline-terminated `start`, `stop`, and `status` at 115200
baud. Replies begin with `#`; sample lines contain two integers separated by a
comma. USB serial is independent of the D7 HX711 data connection.

## Threshold highlighting

Set Threshold (raw ADC counts) to any signed ADC value, including negative values.
With Highlight above threshold enabled, samples strictly greater than the threshold
and the portions of the connecting line above it are red. A dashed red line shows
the threshold. Equal values are not highlighted. Change the threshold after stopping
or loading a CSV to redraw immediately; changes during recording apply to the final
graph. PNG export includes highlighting. Original CSV readings remain unchanged.
