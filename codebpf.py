import FreeSimpleGUI as sg
import time
import serial
import sys
import statistics
import threading
import os
import numpy as np                      # ✅ For numerical operations
from scipy.signal import butter, filtfilt  # ✅ For band-pass filtering

sg.theme('DarkTeal2')

# ----------------------------------------------------------
# Band-pass filter function: cleans ECG/pulse waveform
# ----------------------------------------------------------
def bandpass_filter(data, lowcut=0.5, highcut=4.0, fs=50, order=3):
    """
    Apply a Butterworth band-pass filter to remove baseline drift and high-frequency noise.

    Parameters
    ----------
    data : list or np.ndarray
        Input raw samples (e.g., 50 per packet).
    lowcut : float
        Lower cutoff frequency in Hz (remove slow drift).
    highcut : float
        Upper cutoff frequency in Hz (remove high noise).
    fs : float
        Sampling frequency (Hz) — here 50 Hz from your microcontroller code.
    order : int
        Filter order — higher gives sharper roll-off.

    Returns
    -------
    np.ndarray
        Filtered signal, same length as input.
    """
    # Normalize cutoff frequencies to Nyquist (half the sampling rate)
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq

    # Create a Butterworth band-pass filter
    b, a = butter(order, [low, high], btype="band")

    # Apply the filter using zero-phase filtering (no phase shift)
    y = filtfilt(b, a, data)

    # Return filtered samples as integers (since GUI expects ints)
    return y


# ------------------- SERIAL SETUP -------------------
portName = 'COM11'   # adjust for your system
baudrate = 115200

serialPort = serial.Serial()
serialPort.port = portName
serialPort.baudrate = baudrate
serialPort.bytesize = 8
serialPort.timeout = 1
serialPort.stopbits = serial.STOPBITS_ONE

try:
    serialPort.open()
except Exception as e:
    print(f"Port open failed: {portName} -> {e}")
    sys.exit(1)

print(f"✅ Serial port opened: {portName}")

# ------------------- INITIAL VARIABLES -------------------
bpm = 0.0
avgbpm = 0.0
bpm_values = []
time_values = []
max_points = 100
last_packet_time = time.time()
bluetooth_connected = True

packet_seq = -1

# ------------------- GUI SETUP -------------------
ecg_frame = sg.Frame(
    "ECG Signal (waveform, 50 samples per update)",
    [[sg.Graph(
        canvas_size=(400, 120),
        graph_bottom_left=(0, 0),
        graph_top_right=(400, 120),
        background_color="white",
        key="-GRAPH-",
        border_width=1
    )]],
    relief=sg.RELIEF_SOLID,
    size=(400, 120)
)

bpm_frame = sg.Frame(
    "BPM",
    [[sg.Text(f"{bpm:.1f}", key="-BPM-", font=("Calibri", 50, "bold"), justification='center')]],
    size=(200, 120),
    relief=sg.RELIEF_SOLID
)

status_frame = sg.Frame(
    "Pulse Status",
    [
        [sg.Text("Current Status:", font=("Calibri", 11)), sg.Text(key="-STATUS-", font=("Calibri", 11, "bold"))],
        [sg.Text("Low threshold:"), sg.InputText("60", key="-LOW-", size=(6, 1))],
        [sg.Text("High threshold:"), sg.InputText("100", key="-HIGH-", size=(6, 1))],
        [sg.Button('Apply')]
    ],
    relief=sg.RELIEF_SOLID,
    size=(400, 125)
)

mean_frame = sg.Frame(
    "Mean BPM",
    [[sg.Text(f"{avgbpm:.1f}", key="-MEAN-", font=("Calibri", 25, "bold"), justification='center')]],
    relief=sg.RELIEF_SOLID,
    size=(200, 60)
)

bluetooth_frame = sg.Frame(
    "Bluetooth Status",
    [[sg.Text('Connected', key='-BTSTATUS-', font=('Calibri', 11, 'bold'), text_color='green')]],
    relief=sg.RELIEF_SOLID,
    size=(200, 60)
)

log_frame = sg.Frame(
    "Log",
    [[sg.Multiline(size=(80, 10), key="-LOG-", disabled=True, autoscroll=True)]],
    relief=sg.RELIEF_SOLID,
    size=(620, 200)
)

tab1_layout = [
    [sg.Column([[ecg_frame]], expand_x=True), sg.Column([[bpm_frame]])],
    [sg.Column([[status_frame]]), sg.Column([[mean_frame], [bluetooth_frame]])],
    [sg.Column([[log_frame]])]
]

tab2_layout = [
    [sg.Text("System Information", font=("Calibri", 14, "bold"))],
    [sg.Multiline(
        "Device: PULSE-IT v2.3\nBattery: Unknown\nSignal Strength: Unknown\nFirmware: 1.0.5",
        size=(90, 10),
        key="-SYSINFO-",
        disabled=True,
        autoscroll=True
    )],
    [sg.Button("Generate Report"), sg.Button("Save Log"), sg.Button("Save To Device")]
]

layout = [
    [sg.Text('PULSE 💖 IT', font=('Calibri', 30), justification='center', expand_x=True)],
    [sg.TabGroup([[sg.Tab("Monitoring", tab1_layout), sg.Tab("Reports & System Info", tab2_layout)]])],
    [sg.Button("Exit")]
]

# ------------------- CREATE WINDOW -------------------
window = sg.Window('BMET2922', layout, finalize=True)
graph = window["-GRAPH-"]

# ------------------- ECG GRAPH STARTUP (static baseline) -------------------
for x in range(0, 400, 40):
    graph.draw_line((x, 60), (x + 20, 90), width=1)
    graph.draw_line((x + 20, 90), (x + 40, 60), width=1)

# ------------------- HELPER: ATTEMPT RECONNECT -------------------
def try_reconnect():
    global serialPort, bluetooth_connected
    try:
        if serialPort.is_open:
            serialPort.close()
        time.sleep(1.0)
        serialPort.open()
        bluetooth_connected = True
        window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> 🔁 Reconnected serial port")
        window["-BTSTATUS-"].update("Connected", text_color="green")
    except Exception:
        window["-BTSTATUS-"].update("Disconnected", text_color="red")
        pass

# ------------------- MAIN LOOP -------------------
while True:
    event, values = window.read(timeout=100)
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    current_time = time.time()

    # ----------- READ FROM BLUETOOTH -----------
    try:
        if serialPort.in_waiting > 0:
            line = serialPort.readline().decode('ascii', errors='ignore').strip()
            if not line:
                pass
            else:
                if line.startswith("DATA,"):
                    parts = line.split(",")
                    if len(parts) >= 3 + 50:
                        try:
                            seq = int(parts[1])
                            new_bpm = float(parts[2])
                            raw_samples = parts[3:3 + 50]
                            samples_int = list(map(int, raw_samples))

                            # ----------------------------------------------------------
                            # Apply band-pass filter to clean the waveform before display
                            # ----------------------------------------------------------
                            try:
                                filtered_samples = bandpass_filter(np.array(samples_int))
                                samples_int = list(map(int, filtered_samples))
                            except Exception as e:
                                window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> Filter error: {e}")

                            # Store packet info
                            packet_seq = seq
                            bpm = new_bpm

                            bpm_values.append(bpm)
                            if len(bpm_values) > max_points:
                                bpm_values.pop(0)
                            avgbpm = round(statistics.mean(bpm_values), 1)

                            window["-BPM-"].update(f"{bpm:.1f}")
                            window["-MEAN-"].update(f"{avgbpm:.1f}")

                            try:
                                low_thr = int(values["-LOW-"])
                                high_thr = int(values["-HIGH-"])
                            except:
                                low_thr, high_thr = 60, 100

                            if bpm < low_thr:
                                window["-STATUS-"].update("LOW", text_color="white")
                                window["-LOG-"].print(f"{time.strftime('%a %b %d %H:%M:%S %Y')}: Pulse Low")
                            elif bpm > high_thr:
                                window["-STATUS-"].update("HIGH", text_color="red")
                                window["-LOG-"].print(f"{time.strftime('%a %b %d %H:%M:%S %Y')}: Pulse High")
                            else:
                                window["-STATUS-"].update("NORMAL", text_color="green")

                            window["-LOG-"].print(f"{time.strftime('%a %b %d %H:%M:%S %Y')}: Seq {seq} BPM {bpm:.1f}")

                            # ----------- DRAW FILTERED WAVEFORM -----------
                            graph.erase()
                            min_s, max_s = min(samples_int), max(samples_int)
                            span = (max_s - min_s) or 1
                            scale_x = 400.0 / (len(samples_int) - 1)
                            scale_y = 100.0 / span
                            base_y = 10
                            for i in range(1, len(samples_int)):
                                x1 = (i - 1) * scale_x
                                x2 = i * scale_x
                                y1 = base_y + (samples_int[i - 1] - min_s) * scale_y
                                y2 = base_y + (samples_int[i] - min_s) * scale_y
                                graph.draw_line((x1, y1), (x2, y2))

                            last_packet_time = current_time
                            if not bluetooth_connected:
                                bluetooth_connected = True
                                window["-BTSTATUS-"].update("Connected", text_color="green")

                        except ValueError:
                            window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> Malformed DATA packet ignored: {line}")
                    else:
                        window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> Incomplete DATA (len={len(parts)}), ignored")
                else:
                    if line.startswith("STATUS"):
                        window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> {line}")
    except Exception as e:
        window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> Serial read error: {e}")

    # ----------- BLUETOOTH TIMEOUT CHECK -----------
    if not serialPort.is_open:
        if bluetooth_connected:
            bluetooth_connected = False
            window["-BTSTATUS-"].update("Disconnected", text_color="red")
            window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> ⚠ Serial port closed or lost")
    else:
        if bluetooth_connected and (current_time - last_packet_time > 5):
            bluetooth_connected = False
            window["-BTSTATUS-"].update("Disconnected", text_color="red")
            window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> ⚠ No data received in 5 seconds")

    # ----------- RECONNECT IF DISCONNECTED -----------
    if not bluetooth_connected and (time.time() - last_packet_time) > 3:
        threading.Thread(target=try_reconnect, daemon=True).start()
        last_packet_time = time.time()

    # ----------- HANDLE BUTTONS -----------
    if event == 'Apply':
        try:
            low_threshold = int(values["-LOW-"])
            high_threshold = int(values["-HIGH-"])
            if high_threshold <= low_threshold:
                sg.popup_error("High threshold must be greater than low threshold.", title='Error Message')
                continue
            window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> Thresholds set: low={low_threshold}, high={high_threshold}")
        except ValueError:
            sg.popup_error("Please enter valid integer values for thresholds.", title='Error Message')

    if event == "Generate Report":
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        log_history = values["-LOG-"].strip() or "No log entries recorded."
        report = (
            f"=== PULSE IT SESSION REPORT ===\n"
            f"Timestamp: {timestamp}\n"
            f"Current BPM: {bpm:.1f}\n"
            f"Mean BPM: {avgbpm:.1f}\n"
            f"Low Threshold: {values['-LOW-']}\n"
            f"High Threshold: {values['-HIGH-']}\n"
            f"Status: {values.get('-STATUS-', 'UNKNOWN')}\n"
            f"Bluetooth: {'Connected' if bluetooth_connected else 'Disconnected'}\n"
            f"-----------------------------\n\n"
            f"=== LOG HISTORY ===\n{log_history}\n"
        )
        window["-SYSINFO-"].update(report)
        sg.popup("Report generated. You can Save Log or Save To Device.", title="Report")

    if event == "Save Log":
        filename = sg.popup_get_file("Save Log As", save_as=True, default_extension=".txt",
                                     file_types=(("Text Files", "*.txt"),))
        if filename:
            with open(filename, "w") as f:
                f.write(values["-LOG-"])
            sg.popup("Log saved successfully!", title="Save Log")

    if event == "Save To Device":
        try:
            if serialPort.is_open:
                serialPort.write(b"CMD,SAVE\n")
                window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> Sent CMD,SAVE to device")
            else:
                window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> Cannot send CMD,SAVE, serial closed")
        except Exception as e:
            window["-LOG-"].print(f"{time.strftime('%H:%M:%S')} -> Error sending CMD,SAVE: {e}")

window.close()
try:
    serialPort.close()
except:
    pass
