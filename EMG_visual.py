# visualizer.py
# Receives predicted gesture via UDP and displays it
#not working well

import socket
import pygame

GESTURES = [
    (0, "Hand Close"),
    (1, "Hand Open"),
    (2, "No Motion"),
    (3, "Wrist Extension"),
    (4, "Wrist Flexion"),
]
GESTURE_UDP_PORT = 12347

width, height = 1500, 1500
pygame.init()
window = pygame.display.set_mode([width, height])
pygame.display.set_caption('EMG Gesture Visualizer')
clock = pygame.time.Clock()
font = pygame.font.Font(None, 48)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', GESTURE_UDP_PORT))
sock.setblocking(False)

pred = None

def draw(pred):
    window.fill((50, 50, 50))
    font2 = pygame.font.Font(None, 36)
    window.blit(font2.render("Hand Open", True, (255, 255, 255)), (7*width//16, height//4))
    window.blit(font2.render("Hand Close", True, (255, 255, 255)), (7*width//16, 3*height//4))
    window.blit(font2.render("Wrist Extension", True, (255, 255, 255)), (3*width//4, height//2))
    window.blit(font2.render("Wrist Flexion", True, (255, 255, 255)), (width//8, height//2))
    green = (5, 255, 0)
    if pred == 0:
        pygame.draw.circle(window, green, (width//2, 5*height//8 - 30), 30)
    elif pred == 1:
        pygame.draw.circle(window, green, (width//2, 3*height//8 + 30), 30)
    elif pred == 2:
        pygame.draw.circle(window, green, (width//2, height//2), 30)
    elif pred == 3:
        pygame.draw.circle(window, green, (5*width//8 - 30, height//2), 30)
    elif pred == 4:
        pygame.draw.circle(window, green, (3*width//8 + 30, height//2), 30)
    if pred is not None:
        g_id, g_name = GESTURES[int(pred)]
        surf2 = font.render(f"Predicted: {g_name}", True, (0,255,0))
        window.blit(surf2, (20, 70))
    pygame.display.flip()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    try:
        data, _ = sock.recvfrom(1024)
        pred = int(data.decode('utf-8').strip())
    except BlockingIOError:
        pass
    draw(pred)
    clock.tick(30)
pygame.quit()
