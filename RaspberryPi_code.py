import cv2
import numpy as np
import pyzbar.pyzbar as qr
import smtplib
import os, sys, time
from picamera import PiCamera
from picamera.array import PiRGBArray
from edge_impulse_linux.image import ImageImpulseRunner
import serial
import math

ser=serial.Serial('/dev/ttyACM0',9600)

gmail_user = 'oussamajouini101@gmail.com'
gmail_password = '232613232613'               # no need to try it ! that's not my actual password ;)
sent_from = gmail_user
to = 'oussemajouini@ieee.org'
subject = 'WISY-RACE-RESULT'
body = "la solution est "
result = 0
cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_PLAIN

# Settings
model_file = "modelfile.eim"             # Trained ML model from Edge Impulse
res_width = 96                          # Resolution of camera (width)
res_height = 96                         # Resolution of camera (height)
rotation = 0                            # Camera rotation (0, 90, 180, or 270)

# The ImpulseRunner module will attempt to load files relative to its location,
# so we make it load files relative to this program instead
dir_path = os.path.dirname(os.path.realpath(__file__))
model_path = os.path.join(dir_path, model_file)

# Load the model file
runner = ImageImpulseRunner(model_path)

# Initialize model (and print information if it loads)
try:
    model_info = runner.init()
    print("Model name:", model_info['project']['name'])
    print("Model owner:", model_info['project']['owner'])
    
# Exit if we cannot initialize the model
except Exception as e:
    print("ERROR: Could not initialize model")
    print("Exception:", e)
    if (runner):
            runner.stop()
    sys.exit(1)

# Initial framerate value


def sendEmail(sent_from, subject, sent_to, result):
    email_text = """\
From: %s
To: %s
Subject: %s

%s
""" % (sent_from, ", ".join(sent_to), subject, result)
    
    try:
          smtp_server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
          smtp_server.ehlo()
          smtp_server.login(gmail_user, gmail_password)
          smtp_server.sendmail(sent_from, sent_to, email_text)
          smtp_server.close()
          print ("Email sent successfully!")
    except Exception as ex:
          print ("Something went wrong….",ex)


def scanQr():
    fr = ""
    while(fr == ''):
        ret,frame = cap.read()
        flipped = cv2.flip(frame, flipCode=-1)
        frame1 = cv2.resize(flipped,(640,480))
        qrdetect = qr.decode(frame1)
      
        for i in qrdetect:
            print (i.rect.left,i.rect.top,i.rect.width,i.rect.height)
            fr = str(i.data)
            print (fr)
            print(type(fr))
            cv2.rectangle(frame1,(i.rect.left,i.rect.top),(i.rect.left+i.rect.width,i.rect.top+i.rect.height),(0,255,0),3)
            cv2.putText(frame1,str(i.data),(20,20),font,2,(255,0,0),2) 
        cv2.imshow("Frame", frame1)
        key = cv2.waitKey(1) & 0xFF
    return fr


def getNumber():
    # Start the camera
    counter = 0
    AccX = 0
    fps = 0
    with PiCamera() as camera:
        
        # Configure camera settings
        camera.resolution = (res_width, res_height)
        camera.rotation = rotation
        
        # Container for our frames
        raw_capture = PiRGBArray(camera, size=(res_width, res_height))

        # Continuously capture frames (this is our while loop)
        for frame in camera.capture_continuous(raw_capture, 
                                                format='bgr', 
                                                use_video_port=True):
                                                
            # Get timestamp for calculating actual framerate
            timestamp = cv2.getTickCount()
            
            # Get Numpy array that represents the image
            img = frame.array
            
            # Convert image to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # Encapsulate raw image values into array for model input
            features, cropped = runner.get_features_from_image(img)
            
            # Perform inference
            res = None
            try:
                res = runner.classify(features)
            except Exception as e:
                print("ERROR: Could not perform inference")
                print("Exception:", e)
                
            # Display predictions and timing data
            print("-----")
            results = res['result']['classification']
            for label in results:
                prob = results[label]
                #print(label + ": " + str(round(prob, 3)))
            #print("FPS: " + str(round(fps, 3)))
            
            # Find label with the highest probability
            max_label = max(results, key=results.get)
            print("the number is",max_label)
            
            x=bytes(max_label,'utf-8')
            ser.write(x)
            time.sleep(0.1)

            # Draw max label on preview window
            cv2.putText(img,
                        max_label,
                        (0, 12),
                        cv2.FONT_HERSHEY_PLAIN,
                        1,
                        (255, 255, 255))
                        
            # Draw max probability on preview window
            cv2.putText(img,
                        str(round(results[max_label], 2)),
                        (0, 24),
                        cv2.FONT_HERSHEY_PLAIN,
                        1,
                        (255, 255, 255))
            
            # Show the frame
            cv2.imshow("Frame", img)
            
            # Clear the stream to prepare for next frame
            raw_capture.truncate(0)
            
            # Calculate framrate
            frame_time = (cv2.getTickCount() - timestamp) / cv2.getTickFrequency()
            fps = 1 / frame_time
            
            #counter = counter + 1
            break
            #AccX = AccX + int.from_bytes(x, "big")
            #if (counter >= 10):
            #    if(AccX / 10 == int.from_bytes(x, "big")):
            #        break
            #    else:
            #        AccX = int.from_bytes(x, "big")
            
    # Clean up
    cv2.destroyAllWindows()
    return x


def getDeltaResults(a, b, c):

    aa=int.from_bytes(a,"big")
  
    bb=int.from_bytes(b,"big")
    cc=int.from_bytes(c,"big")
    delta = bb-4*aa*cc

    racineDeDelta=math.sqrt(abs(delta))
    retour = [(-bb-racineDeDelta)/(2*aa),(-bb+racineDeDelta)/(2*aa)]

    return retour


while True:
    receivedChar = ser.read()
    time.sleep(0.01)
    
    
    if (receivedChar == b'C'):
        print(receivedChar)
        c = getNumber()
        ser.write(c)
    elif (receivedChar == b'A'):
        print(receivedChar)
        a = getNumber()
        ser.write(a)
    elif (receivedChar == b'B'):
        print(receivedChar)
        b = getNumber()
        ser.write(b)
    elif (receivedChar == b'Q'):
        print(receivedChar)
        adress = scanQr()
        if(adress.find("'")):
            adress = adress[2:len(adress)-1]
        result = getDeltaResults(a,b,c)
        ser.write(result)
        sendEmail(sent_from, subject, adress, result)
    
    receivedChar = ""
    
    
