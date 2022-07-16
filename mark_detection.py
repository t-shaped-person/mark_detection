import cv2
import sys
import time
import serial

from raspberry_pi_camera.simple_camera import gstreamer_pipeline
from aruco_markers.utils import ARUCO_DICT, aruco_display


def main():
    aruco_markers_type = 'DICT_5X5_100' # if you need change~~
    # uart = serial.Serial('/dev/ttyUSB0', 115200) # USB to serial
    uart = serial.Serial('/dev/ttyTHS1', 115200) # JETSON serial
    # video = cv2.VideoCapture(0) # USB cam
    video = cv2.VideoCapture(gstreamer_pipeline(capture_width=640, capture_height=360, framerate=30), cv2.CAP_GSTREAMER) # raspberry pi camera
    time.sleep(2.0)

    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_markers_type])
    arucoParams = cv2.aruco.DetectorParameters_create()

    while True:
        ret, frame = video.read()
        if ret is False:
            break
        h, w, _ = frame.shape
        width=1000
        height = int(width*(h/w))
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_CUBIC)
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        send_data = '$'
        if len(corners):
            send_data = send_data + str(len(corners)) + ','
            for i in range(len(corners)): # 0:좌상, 1:우상, 2:우하, 3:좌하
                left_up = corners[i][0][0]
                right_up = corners[i][0][1]
                right_down = corners[i][0][2]
                left_down = corners[i][0][3]
                no = ids[i][0]
                w_c = int(left_up[0] + (right_up[0] - left_up[0]) / 2)
                h_c = int(left_up[1] + (left_down[1] - left_up[1]) / 2)
                send_data = send_data + str(no) + ',' + str(w_c) + ',' + str(h_c) + ','
            send_data = send_data[0:-1]
            send_data = send_data + '\r' + '\n'
            uart.write(bytes(send_data, encoding='ascii'))
        else:
            send_data = send_data + '0'
            time.sleep(0.05)
            uart.write(bytes(send_data, encoding='ascii'))

        detected_markers = aruco_display(corners, ids, rejected, frame) # 디버깅용 화면 표시
        cv2.imshow("Image", detected_markers) # 디버깅용 화면 표시
        # time.sleep(0.05) # 송부 시간 제어, 초단위
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    cv2.destroyAllWindows()
    video.release()
    uart.close()

if __name__ == '__main__':
    main()