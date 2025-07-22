# train_udp.py
# Collects EMG data and gesture labels via UDP for ML training

import socket
import pygame
from collections import deque
import numpy as np
import time
import os
import csv

GESTURES = [
    (0, "Hand Close"),
    (1, "Hand Open"),
    (2, "No Motion"),
    (3, "Wrist Extension"),
    (4, "Wrist Flexion"),
]

UDP_PORT = 12346
DATA_DIR = "data"
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)

class EMGTrainer:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', UDP_PORT))
        self.sock.setblocking(False)
        self.width, self.height = 800, 600
        pygame.init()
        self.window = pygame.display.set_mode([self.width, self.height])
        pygame.display.set_caption('EMG Training Collector')
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 48)
        self.running = True
        self.current_label = 0
        self.samples = []
        self.label_names = [g[1] for g in GESTURES]
        self.instructions = "Press 0-4 to select gesture, S to save, Q to quit"

    def draw(self):
        self.window.fill((30, 30, 30))
        surf = self.font.render(f"Current: {self.label_names[self.current_label]}", True, (255,255,0))
        self.window.blit(surf, (20, 20))
        surf2 = self.font.render(self.instructions, True, (200,200,200))
        self.window.blit(surf2, (20, 80))
        pygame.display.flip()

    def run(self):
        print("EMG Trainer started. Press 0-4 to select gesture, S to save, Q to quit.")
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        self.running = False
                    elif event.key == pygame.K_s:
                        self.save()
                    elif pygame.K_0 <= event.key <= pygame.K_4:
                        self.current_label = event.key - pygame.K_0
            try:
                data, _ = self.sock.recvfrom(1024)
                decoded = data.decode("utf-8").strip()
                parts = decoded.split()
                if len(parts) >= 8:
                    emg = [int(x) for x in parts[:8]]
                    self.samples.append((emg, self.current_label))
            except BlockingIOError:
                pass
            self.draw()
            self.clock.tick(30)
        pygame.quit()
        print("Exiting trainer.")

    def save(self):
        timestamp = int(time.time())
        filename = os.path.join(DATA_DIR, f"emg_train_{timestamp}.csv")
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["emg0","emg1","emg2","emg3","emg4","emg5","emg6","emg7","label"])
            for emg, label in self.samples:
                writer.writerow(list(emg) + [label])
        print(f"Saved {len(self.samples)} samples to {filename}")
        self.samples = []

if __name__ == "__main__":
    EMGTrainer().run()
