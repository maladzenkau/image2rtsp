import cv2

cap = cv2.VideoCapture('rtsp://0.0.0.0:8554/rs')

while True:
    ret, frame = cap.read()

    if ret:
        cv2.imshow('RTSP Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
