import cv2                  # Thu vien xu ly anh
import numpy as np          # Thu vien tinh toan mang va ma tran
import serial               # Thu vien giao tiep voi Serial
import time                 # Thu vien thoi gian (delay)

# Thu ket noi voi Arduino
try:
    arduino = serial.Serial("COM3", 115200, timeout=0.1)        # Ket noi cong COM3, baudrate 115200
    arduino_connected = True                                    # Ket noi thanh cong
    print("Connected Arduino COM3.")
    time.sleep(2)                                               # Cho Arduino khoi dong
except serial.SerialException:
    arduino_connected = False                                   # Ket noi khong thanh cong 
    print("Arduino not connected. Running in test mode.")    

# Khoi tao camera
cap = cv2.VideoCapture(0)                                       # Mo webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)                          # Chieu rong khung anh
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)                         # Chieu cao khung anh

# HSV nguong mau xanh la
lower_green = np.array([40, 100, 100])                          # Nguong duoi
upper_green = np.array([80, 255, 255])                          # Nguong tren

# Goc ban dau
servo_x, servo_y = 90, 90                                       # Goc ban dau
alpha = 0.3                                                     # He so lam muot
gain = 0.05                                                     # He so phan hoi

# Trung tam anh
frame_center_x = 640 // 2                                       # Trung tam anh theo truc x
frame_center_y = 480 // 2                                       # Trung tam anh theo truc y

while True:
    ret, frame = cap.read()                                     # Doc frame tu camera
    if not ret:
        break                                                   # Neu khong co hinh thi thoat

    frame = cv2.flip(frame, 1)                                  # Lat anh ngang
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)                # Chuyen anh sang khong gian mau HSV
    mask = cv2.inRange(hsv, lower_green, upper_green)           # Tao mask loc mau xanh la

    # Khu nhieu

    mask = cv2.GaussianBlur(mask, (7, 7), 0)                    # Lam mo de giam nhieu
    mask = cv2.erode(mask, None, iterations=2)                  # Bao mon canh
    mask = cv2.dilate(mask, None, iterations=2)                 # Gian vung de lam day


    # Xac dinh contour
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    # Tim duong bao ngoai

    if contours:
        c = max(contours, key=cv2.contourArea)                                          # Lay contour lon nhat
        ((x, y), radius) = cv2.minEnclosingCircle(c)                                    # Tim duong tron bao quanh contour

        if radius > 5:                                                                  # Chi xu ly neu vat du lon
            # Ve vung nhan dien
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)            # Ve duong tron bao quanh
            cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)                     # Ve cham do tai tam
            cv2.putText(frame, f"Target: ({int(x)}, {int(y)})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)              # Hien thi toa do doi tuong

            # Tinh sai so
            error_x = frame_center_x - x                                                # Sai so theo truc x
            error_y = frame_center_y - y                                                # Sai so theo truc y

            # Chuyen sai so sang goc
            if abs(error_x) > 10:
                servo_x_target = servo_x - (error_x * gain)                             # Chuyen doi goc servo x
            if abs(error_y) > 10:
                servo_y_target = servo_y + (error_y * gain)                             # Chuyen doi goc servo y


            # Lam muot tranh dao dong manh
            servo_x = int(alpha * servo_x_target + (1 - alpha) * servo_x)               # Lam muot goc servo x
            servo_y = int(alpha * servo_y_target + (1 - alpha) * servo_y)               # Lam muot goc servo y

            # Gioi han goc
            servo_x = max(0, min(180, servo_x))                                         # Gioi han goc servo x
            servo_y = max(60, min(180, servo_y))                                        # Gioi han goc servo y

            # Gui du lieu neu da ket noi Arduino
            if arduino_connected:
                data = f"{servo_x},{servo_y}\n"                                         # Dong goi du lieu thanh chuoi
                arduino.write(data.encode())                                            # Gui chuoi qua Serial

            # Hien thi goc servo
            cv2.putText(frame, f"Servo X: {servo_x}", (10, 60),                         
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Servo Y: {servo_y}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Hien thi khung hinh
    cv2.imshow("Green Tracking", frame)                                                 # Hien thi hinh anh goc
    cv2.imshow("Mask", mask)                                                            # Hien thi vung mau da loc

    if cv2.waitKey(1) == 27:                                                            # Nhan ESC de thoat
        break

cap.release()                                                                           # Giai phong camera
cv2.destroyAllWindows()                                                                 # Dong tat ca cua so
if arduino_connected:
    arduino.close()                                                                     # Ngat ket noi Arduino neu dang ket noi
