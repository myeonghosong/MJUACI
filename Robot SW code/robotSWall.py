import cv2
import numpy as np
import copy
import paho.mqtt.client as mqtt
import time
import spidev
import sys

SystemAllFlag = 0
frame = None
t = None
t_cir = None
f = None
b = None
gray = None
gray_cir = None
binary = None
binary_cir = None
Circle = None
cnt_x = 0
cnt_y = 0
cnt_x_cir = 0
cnt_y_cir = 0
parallel = 0
flagMQTT = 0
Zflag = 0
XFlag=0
sublight = 0
light = 60
YYY = 0
ZZZ = 0
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000

def readChannel(channel):
    val = spi.xfer2([1,(8+channel)<<4,0])
    data = ((val[1]&3)<<8)+val[2]
    return data

def on_message(client, userdata, message): #MQTT콜백
    global RobotStep, light,YYY, ZZZ,SystemAllFlag

    print("message received : ", str(message.payload.decode("utf-8")))
    print("message topic= ", message.topic)
    print("message qos= ", message.qos)
    print("message retain flag= ", message.retain)
    if message.topic == "MJU/CD4/CHARGING":
        if str(message.payload.decode("utf-8")) == "START":
            RobotStep = 1
            print("success")

    if message.topic == "MJU/CD4/CHARGING/LIGHT":
        if str(message.payload.decode("utf-8")) == "-":
            light = light - 10
            mqtt.publish("MJU/CD4/CHARGING/LIGHT",light)
        elif str(message.payload.decode("utf-8")) == "+":
            light = light + 10
            mqtt.publish("MJU/CD4/CHARGING/LIGHT",light)
        elif str(message.payload.decode("utf-8")) == "START":
            YYY = 1
        elif str(message.payload.decode("utf-8")) == "STOP":
            YYY = 0
            ZZZ = 0

    if message.topic == "MJU/CD4/CHARGING/CAR":
        if str(message.payload.decode("utf-8")) == "IN": #차량 진입->해당위치로 이동 단계
            SystemAllFlag = 1
        elif str(message.payload.decode("utf-8")) == "FindCARNUM": #번호판 인식단계
            SystemAllFlag = 2
        elif str(message.payload.decode("utf-8")) == "FindConnector": #커넥터 찾아서 잡는 단계
            SystemAllFlag = 3
        elif str(message.payload.decode("utf-8")) == "GoToPort": #차량 충전구로 이동 단계
            SystemAllFlag = 4
        elif str(message.payload.decode("utf-8")) == "FindPort": #충전구 탐색 및 삽입 단계
            SystemAllFlag = 5
        elif str(message.payload.decode("utf-8")) == "Success": #충전연결 성공, 커넥터 꽂은채로 로봇팔 회수 단계
            SystemAllFlag = 6
        elif str(message.payload.decode("utf-8")) == "BackForWaiting": #로봇 원래자리로 돌아가는 단계
            SystemAllFlag = 7
        elif str(message.payload.decode("utf-8")) == "WaitingCharge": #충전완료까지 대기하는 단계
            SystemAllFlag = 8
        elif str(message.payload.decode("utf-8")) == "StopCharging": #충전정지신호 받고 로봇 충전구로 이동하는 단계
            SystemAllFlag = 9
        elif str(message.payload.decode("utf-8")) == "PullConnector": #커넥터 찾아서 잡고 전자서 켜서 뽑는 단계
            SystemAllFlag = 10
        elif str(message.payload.decode("utf-8")) == "PutConnector": #커넥터 돌려놓으면서 회수시스템 작동 단계
            SystemAllFlag = 11
        elif str(message.payload.decode("utf-8")) == "CARINWaiting": #원점으로 돌아가는 단계 끝나면 SystemAllFlag = 0으로 돌아가서 반복
            SystemAllFlag = 12
def center_xy_Circle(flag):
    global frame, gray_cir, binary_cir, cnt_x_cir, cnt_y_cir, Circle, light

    gray_cir = cv2.cvtColor(Circle, cv2.COLOR_BGR2GRAY)

    print("--------------------------")
    print(gray_cir.mean())
    
    #if(abs(140 - gray_cir.mean()) > 0):
    #    gray_cir = gray_cir + abs(140 - int(gray_cir.mean()))

    t_cir = cv2.bilateralFilter(gray_cir, -1, 10, 5)
    _, binary_cir = cv2.threshold(t_cir, light, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_cir, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if(flag == 1):
        
        first = 0
        last = 0
        cnt = 0
        centers = []
        radius = np.array([])  # Initialize radius as an empty NumPy array
        for contour in contours:
            if len(contour) < 5:
                continue

            area = cv2.contourArea(contour)
            if area < 1 or area > 500:
                continue

            (x, y), rad = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = np.append(radius, rad) 

            cv2.circle(Circle, center, int(rad), (0, 0, 255), 2, cv2.LINE_AA)
            cv2.circle(Circle, center, 10, (0, 0, 255), -1, cv2.LINE_AA)

            centers.append(center)

        if len(centers) >0:
            cnt_x_cir = 0
            cnt_y_cir = 0
            for center in centers:
                cnt_x_cir += centers[0][0]
                cnt_y_cir += centers[0][1]
                cv2.circle(Circle, center, 10, (0, 255, 0), -1, cv2.LINE_AA)
            cnt_x_cir /= len(centers)
            cnt_y_cir /= len(centers)


def center_xy_Rect(flag):
    global frame, gray, binary, cnt_x, cnt_y, frame_x, frame_y,light

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    print("--------------------------")
    print(gray.mean())
    
    #if(abs(140 - gray.mean()) > 0):
    #    gray = gray + abs(140 - int(gray.mean()))

    if(flag == 1):
        t = cv2.bilateralFilter(gray, -1, 10, 5)
        _, binary = cv2.threshold(t, light, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #윤곽선 검출

        centers = []
        for contour in contours:
            if len(contour) < 4:  
                continue

            area = cv2.contourArea(contour)
            if (area < 3000): #면적 계산, 500보다 작거나 13000보다 크면 윤곽무시
                continue
            
            if (70000 < area): #면적 계산, 500보다 작거나 13000보다 크면 윤곽무시
                continue
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)  #다각형 근사

            if len(approx) == 4:  
                (x, y, w, h) = cv2.boundingRect(approx)
                center = (int(x + w/2), int(y + h/2))
                centers.append(center)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2) #바운딩 박스그리기
                cv2.circle(frame, center, 10, (0, 0, 255), -1) #중심좌표에 원 그리기

        if len(centers) == 1: 
            (x, y) = centers[0]
            cnt_x = x
            cnt_y = y
            cv2.circle(frame, (x, y), 10, (0, 255, 0), -1)
            #cnt_x /= len(centers)
            #cnt_y /= len(centers)

            print(f"검출완료 X좌표: {cnt_x}, Y좌표: {cnt_y}")



def move_center():
    global frame, cnt_x, cnt_y, flagMQTT, Circle, mqtt, Zflag, YYY, ZZZ

    print("중심으로 이동중....")
    flagMQTT += 1
    if (flagMQTT > 2):
        if(Zflag == 0):
            if(YYY == 1):
                mqtt.publish("MJU/IOT/60181793/Y", cnt_x) #토픽과 메세지 발행
            
            if(ZZZ == 1):
                mqtt.publish("MJU/IOT/60181793/Z", cnt_y) #토픽과 메세지 발행
            flagMQTT = 0


#def move_center_Y():
 #   global frame, cnt_x, cnt_y, flagMQTT, Circle, mqtt, Zflag
#
 #   print("중심으로 이동중....")
  #  flagMQTT += 1
   # if (flagMQTT > 2):
    #    if(Zflag == 0):

  


def insertion():
    arm_motor()


def stage_motor(y):
    print("y축 이동중....")


def wheel_motor(x):
    print("x축 이동중....")


def arm_motor():
  
    print("팔 뻗는중....")

#def on_connect(client, userdata, flags, rc):
#	print("Connected with result code "+str(rc))
	# Subscribing in on_connect() means that if we lose the connection and
	# reconnect then subscriptions will be renewed.
#	client.subscribe("MJU/IOT/60181793/Flag")


# The callback for when a PUBLISH message is received from the server.
#def on_message(client, userdata, msg):
#    global flagMQTT
#    print(str(msg.payload))
#    flagMQTT = str(msg.payload)
def motiondetect(im1,im2,im3):
    global flag, gray_img01, gray_img02, gray_img03 , ret
    gray_img01 = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    gray_img02 = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    gray_img03 = cv2.cvtColor(im3, cv2.COLOR_BGR2GRAY)
    diff01 = cv2.absdiff(gray_img01,gray_img02)
    diff02 = cv2.absdiff(gray_img02,gray_img03)

    ret, diff01 = cv2.threshold(diff01, 20, 255, cv2.THRESH_BINARY)
    ret, diff02 = cv2.threshold(diff02, 20, 255, cv2.THRESH_BINARY)
    
    diff = cv2.bitwise_and(diff01, diff02)
    diff_cnt = cv2.countNonZero(diff) # amount of difference(the closer to zero, the less differeceq)

    #cv2.imshow("Motion sensing", diff)
    print(diff_cnt)

    if(diff_cnt > 20): flag = 0
    elif(diff_cnt <= 20): flag = 1



def main():
    global frame, cnt_x, cnt_y, x, y, parallel, Circle, mqtt,flag, ret, img01, img02, img03, Zflag, Xflag, YYY, ZZZ
    flagMQTT = 0

    mqtt = mqtt.Client("python_pub") #Mqtt Client 오브젝트 생성
    mqtt.connect("cheese.mju.ac.kr", 30220) #MQTT 서버에 연결
    mqtt.subscribe("MJU/CD4/CHARGING") #차량 진입 시
    mqtt.subscribe("MJU/CD4/CHARGING/LIGHT")
    mqtt.on_message = on_message
    mqtt.loop_start()
    
    

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open the camera.")
        return

    cap.set(cv2.CAP_PROP_FPS, 8)
    cv2.namedWindow("Real-time video", cv2.WINDOW_NORMAL)
    _, frame = cap.read()
    frame_y = frame.shape[0] / 2  
    frame_x = frame.shape[1] / 2

    while True:

        _, frame = cap.read()
        ret, img01 = cap.read()
        ret, img02 = cap.read()
        ret, img03 = cap.read()
        Circle = copy.deepcopy(frame)

        if frame is None:
            print("Could not read the frame.")
            break
        #time.sleep(1)
        # Start
        motiondetect(img01,img02,img03)
        center_xy_Rect(flag)
        center_xy_Circle(flag)

        if flag == 1 :
            print(frame_x)
            print(frame_y)
            print(cnt_x)
            print(cnt_y)
            #MQTT(cnt_x,cnt_y)

            size = 50
            thickness = 2
            pt1 = (int(frame_x - size / 2), int(frame_y))
            pt2 = (int(frame_x + size / 2), int(frame_y))
            pt3 = (int(frame_x), int(frame_y - size / 2))
            pt4 = (int(frame_x), int(frame_y + size / 2))

            if abs(frame_y - cnt_y) < 4 and cnt_y > 0:
                if parallel == 0:
                    if Zflag == 0:
                        mqtt.publish("MJU/IOT/60181793/Z","9999") #토픽과 메세지 발행
                        ZZZ = 0
                        YYY = 1
                        time.sleep(3)
                        mqtt.publish("MJU/IOT/60181793/X","120")
                        time.sleep(3) 
                        mqtt.publish("MJU/IOT/60181793/X","60")               #        Zflag = 1
                #    move_slope()
                #    cv2.line(Circle, pt1, pt2, (255, 0, 0), thickness)
                #    cv2.line(Circle, pt3, pt4, (255, 0, 0), thickness)
                #if parallel == 1:
                #    insertion()
                #    cv2.line(Circle, pt1, pt2, (0, 255, 255), thickness)
                #    cv2.line(Circle, pt3, pt4, (0, 255, 255), thickness)
            elif cnt_y > 0:
                cv2.line(Circle, pt1, pt2, (0, 255, 0), thickness)
                cv2.line(Circle, pt3, pt4, (0, 255, 0), thickness)
                move_center()

            if abs(frame_x+34 - cnt_x) < 1 and cnt_x > 0 :
                if parallel == 0:
                    if Zflag == 0:
                        mqtt.publish("MJU/IOT/60181793/Y","99999") #토픽과 메세지 발행
                        ZZZ = 1
                        YYY = 0
                #       Zflag = 1
                #    move_slope()
                #    cv2.line(Circle, pt1, pt2, (255, 0, 0), thickness)
                #    cv2.line(Circle, pt3, pt4, (255, 0, 0), thickness)
                #if parallel == 1:
                #    insertion()
                #    cv2.line(Circle, pt1, pt2, (0, 255, 255), thickness)
                #    cv2.line(Circle, pt3, pt4, (0, 255, 255), thickness)
            elif cnt_x > 0 :
                cv2.line(Circle, pt1, pt2, (0, 255, 0), thickness)
                cv2.line(Circle, pt3, pt4, (0, 255, 0), thickness)
                move_center()
            
            

                #Zflag = 0
            # End
            #elif cnt_x > 0:
                #cv2.line(Circle, pt1, pt2, (0, 255, 0), thickness)
                #cv2.line(Circle, pt3, pt4, (0, 255, 0), thickness)
                #move_center_X()
                #if abs(frame_x - cnt_x) < 3:
                #    XFlag = 1

           # elif cnt_y > 0 and XFlag == 1:
           #     cv2.line(Circle, pt1, pt2, (0, 255, 0), thickness)
           #     cv2.line(Circle, pt3, pt4, (0, 255, 0), thickness)
            #    move_center_Y()
                #Zflag = 0
            # End

        cv2.imshow("Real-time video", frame)
        cv2.imshow("threshold", binary_cir)
        #cv2.imshow("circle",Circle)
            #time.sleep(0.5)
        if cv2.waitKey(1) == 27:
                break

        print("\n" * 10)  # Clear the console
            #client.loop_forever()
            #client.loop()
            #cv2.waitKey(10)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()