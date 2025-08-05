# ml_classifier.py
# Receives EMG via UDP, predicts gesture, sends gesture via UDP to visualizer
# not optimized

import socket
import os
import numpy as np
from collections import deque
from joblib import load

DATA_DIR = "data"
MODEL_PATH = os.path.join(DATA_DIR, "emg_svm.joblib")
EMG_UDP_PORT = 12346
GESTURE_UDP_PORT = 12347

GESTURES = [
    (0, "Hand Close"),
    (1, "Hand Open"),
    (2, "No Motion"),
    (3, "Wrist Extension"),
    (4, "Wrist Flexion"),
]

def main():
    if not os.path.exists(MODEL_PATH):
        print("No trained model found. Please run training first.")
        return
    model, scaler = load(MODEL_PATH)

    # UDP socket for EMG input
    sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_in.bind(('127.0.0.1', EMG_UDP_PORT))
    sock_in.setblocking(False)

    # UDP socket for gesture output
    sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    cache = deque(maxlen=20)  # Larger cache for more stable voting
    pred = None
    last_pred = None
    stable_count = 0
    min_stable_frames = 8  # Only update gesture if stable for this many frames
    confidence_threshold = 0.6  # Require at least 60% agreement in cache
    emg = np.zeros(8)

    print("ML Classifier running. Waiting for EMG packets...")
    log_file = open('ml_classifier_uncertain.log', 'a')
    while True:
        try:
            data, _ = sock_in.recvfrom(1024)
            decoded = data.decode("utf-8").strip()
            parts = decoded.split()
            if len(parts) >= 8:
                emg = np.array([int(x) for x in parts[:8]])
                # Outlier rejection: ignore if any channel is out of reasonable range
                if np.any(emg > 250) or np.any(emg < -10):
                    continue
                Xs = scaler.transform([emg])
                pred_class = model.predict(Xs)[0]
                cache.append(pred_class)
                # Majority vote with confidence threshold
                values, counts = np.unique(cache, return_counts=True)
                majority_idx = np.argmax(counts)
                majority_pred = values[majority_idx]
                confidence = counts[majority_idx] / len(cache)
                # Dead zone: if confidence is too low, output 'No Motion' (2)
                if confidence < confidence_threshold:
                    # Log uncertain predictions for analysis
                    log_file.write(f"Uncertain: EMG={emg.tolist()} Preds={list(cache)}\n")
                    log_file.flush()
                    pred = 2  # No Motion
                else:
                    if last_pred == majority_pred:
                        stable_count += 1
                    else:
                        stable_count = 1
                        last_pred = majority_pred
                    if stable_count >= min_stable_frames:
                        pred = majority_pred
                # Send prediction to visualizer
                if pred is not None:
                    sock_out.sendto(str(int(pred)).encode('utf-8'), ('127.0.0.1', GESTURE_UDP_PORT))
        except BlockingIOError:
            pass

if __name__ == '__main__':
    main()
