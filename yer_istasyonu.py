"""
An interface to visualize telemetry from UAV at Basic T standards.
"""
import cv2
import imutils
import numpy as np
import dronekit

iha=connect(connection_string,wait_ready=True,timeout=100)
count = 0
onceki_durum = 0

def yer_istasyonu():
    global iha
    global count
    global onceki_durum
    while True:

        # göstergelerin işlenmesi için kopyası oluşturuluyor.
        b1 = back1.copy()
        b2 = back2.copy()
        b3 = back3.copy()
        b4 = back4.copy()
        b5 = back5.copy()
        b6 = back6.copy()


        # telemetri verilerine göre indikatörleri döndürme işlemi
        if iha.velocity[0] > 0 and iha.velocity[0] > onceki_durum :
            count = count + 1
        elif iha.velocity[0] > 0 and iha.velocity[0] < onceki_durum :
            count = count - 1
        elif iha.velocity[0] < 0 and iha.velocity[0] > onceki_durum :
            count = count + 1
        elif iha.velocity[0] < 0 and iha.velocity[0] < onceki_durum :
            count = count - 1
        indicator6 = imutils.rotate_bound(arrow, -(iha.velocity[2]*10)+ 30)
        indicator5 = imutils.rotate_bound(a1, iha.heading*1.5 +90)
        indicator3 = imutils.rotate_bound(arrow, iha.location.global_relative_frame.alt*3.5+117)
        indicator2 = imutils.rotate_bound(i2, iha.velocity[1]*3)
        indicator4 = imutils.rotate_bound(a2, iha.attitude.roll*10)
        indicator1 = imutils.rotate_bound(arrow, iha.airspeed*6.7-10)
        onceki_durum = iha.velocity[0]


        # indikatör ve gösterglerin birleştirilmesi
#1


        x_offset = y_offset = int(b1.shape[0] / 2) - int(indicator1.shape[0] / 2)
        y1, y2 = y_offset, y_offset + indicator1.shape[0]
        x1, x2 = x_offset, x_offset + indicator1.shape[1]
        alpha_s = indicator1[:, :, 3] / 255.0
        alpha_l = 1.0 - alpha_s
        for c in range(0, 3):
            b1[y1:y2, x1:x2, c] = (alpha_s * indicator1[:, :, c] + alpha_l * b1[y1:y2, x1:x2, c])
#2

        x_offset = y_offset = int(b2.shape[0] / 2) - int(indicator2.shape[0] / 2)
        y1, y2 = (y_offset), (y_offset) + indicator2.shape[0]#y_offset+count
        x1, x2 = x_offset, x_offset + indicator2.shape[1]
        alpha_s = indicator2[:, :, 3] / 255.0
        alpha_l = 1.0 - alpha_s
        for c in range(0, 3):
            b2[y1:y2, x1:x2, c] = (alpha_s * indicator2[:, :, c] + alpha_l * b2[y1:y2, x1:x2, c])
#3
        x_offset = y_offset = int(b3.shape[0] / 2) - int(indicator3.shape[0] / 2)
        y1, y2 = y_offset, y_offset + indicator3.shape[0]
        x1, x2 = x_offset, x_offset + indicator3.shape[1]
        alpha_s = indicator3[:, :, 3] / 255.0
        alpha_l = 1.0 - alpha_s
        for c in range(0, 3):
            b3[y1:y2, x1:x2, c] = (alpha_s * indicator3[:, :, c] + alpha_l * b3[y1:y2, x1:x2, c])
#4
        x_offset = y_offset = int(b4.shape[0] / 2) - int(indicator4.shape[0] / 2)
        y1, y2 = y_offset, y_offset + indicator4.shape[0]
        x1, x2 = x_offset, x_offset + indicator4.shape[1]
        alpha_s = indicator4[:, :, 3] / 255.0
        alpha_l = 1.0 - alpha_s
        for c in range(0, 3):
            b4[y1:y2, x1:x2, c] = (alpha_s * indicator4[:, :, c] + alpha_l * b4[y1:y2, x1:x2, c])

#5
        x_offset = y_offset = int(b5.shape[0] / 2) - int(indicator5.shape[0] / 2)
        y1, y2 = y_offset, y_offset + indicator5.shape[0]
        x1, x2 = x_offset, x_offset + indicator5.shape[1]
        alpha_s = indicator5[:, :, 3] / 255.0
        alpha_l = 1.0 - alpha_s
        for c in range(0, 3):
            b5[y1:y2, x1:x2, c] = (alpha_s * indicator5[:, :, c] + alpha_l * b5[y1:y2, x1:x2, c])


#6
        x_offset = y_offset = int(b6.shape[0] / 2) - int(indicator6.shape[0] / 2)
        y1, y2 = y_offset, y_offset + indicator6.shape[0]
        x1, x2 = x_offset, x_offset + indicator6.shape[1]
        alpha_s = indicator6[:, :, 3] / 255.0
        alpha_l = 1.0 - alpha_s
        for c in range(0, 3):
            b6[y1:y2, x1:x2, c] = (alpha_s * indicator6[:, :, c] + alpha_l * b6[y1:y2, x1:x2, c])



        #göstergeler birleştirilir ve çıktı olarak gösterilir
        upper_half = np.concatenate((b1, b2, b3), axis=1)
        lower_half = np.concatenate((b4, b5, b6), axis=1)
        telemetry = np.concatenate((upper_half, lower_half), axis=0)
        cv2.imshow(window_name, telemetry)
        cv2.waitKey(1)
