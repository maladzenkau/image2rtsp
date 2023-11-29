import cv2

cap = cv2.VideoCapture('rtsp://127.0.0.1:8554/back')

while True:
    ret, frame = cap.read()

    if ret:
        cv2.imshow('RTSP Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
