from picamera.array import PiRGBArray
from picamera import PiCamera
import pigpio
import numpy as np
import time
import datetime
import cv2
import RPi.GPIO as GPIO
import math

from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils

servopin = 18
breite, hoehe = (640, 480)

#np.seterr(all="ignore")

#camera = PiCamera()
#breite, hoehe = (320, 240)
#camera.resolution =  (int(breite / 2), int(hoehe / 2)) ######
#camera.framerate = 30
#rawCapture = PiRGBArray(camera, size=(int(breite / 2), int(hoehe / 2))) ##########

#stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)



stream = PiVideoStream().start()

time.sleep(2.0)


relevanz = 0.65
korrektur = 7 #(+ = nach links (um grad))
v = 20

StepPinBackward = 13
StepPinForward = 6
StepPinnnBackward = 26
StepPinnnForward = 19
StepPinPWMRechts = 12
StepPinPWMLinks = 5
ReedVorne = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(StepPinForward, GPIO.OUT)
GPIO.setup(StepPinBackward, GPIO.OUT)
GPIO.setup(StepPinnnForward, GPIO.OUT)
GPIO.setup(StepPinnnBackward, GPIO.OUT)
GPIO.setup(StepPinPWMRechts, GPIO.OUT)
GPIO.setup(StepPinPWMLinks, GPIO.OUT)

GPIO.setup(ReedVorne, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

StepperRechts = GPIO.PWM(StepPinPWMRechts, 100)
StepperLinks = GPIO.PWM(StepPinPWMLinks, 100)
    
StepperRechts.start(0)
StepperLinks.start(0)

winkel_jetzt = 90


pi = pigpio.pi() #connect to local pi

#warmup
time.sleep(0.1)

def forward(winkel, bild, vI, achsenlaenge=30, achsenabstand=23):
    #GPIO.output(StepPinForward, GPIO.HIGH)
    #GPIO.output(StepPinnnForward, GPIO.HIGH)
    korrwinkel = (winkel) - 90

    if korrwinkel < 0:
        dwinkel = korrwinkel * (-1)
        rechtsKurve = True
    else:
        dwinkel = korrwinkel
        rechtsKurve = False
    
    

    #temp = korrwinkel
    #math.tan(winkel * (180 / np.pi))

    
    if dwinkel == 0:
        StepperRechts.ChangeDutyCycle(vI)
        StepperLinks.ChangeDutyCycle(vI)
        
    else:
        vA = vI * (1 + (1 / math.pi) * (1 - ((2 * math.pi * (achsenabstand / math.tan(math.radians(dwinkel)))) / (2 * math.pi * (achsenabstand / math.tan(math.radians(dwinkel)) + achsenlaenge)))))
        #vA = 20
        if rechtsKurve:
            StepperRechts.ChangeDutyCycle(vI)
            StepperLinks.ChangeDutyCycle(vA)
            
        else:
            StepperRechts.ChangeDutyCycle(vA)
            StepperLinks.ChangeDutyCycle(vI)

        cv2.putText(bild, str(vA), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("temnppasd", bild)


def konversionen(bild):
    unteresblau = np.array([90, 35, 35])
    oberesblau = np.array([120, 255, 255])
    unteresgruen = np.array([160, 25, 35])
    oberesgruen = np.array([180, 255, 255])
    unteresrot = np.array([0, 35, 35])
    oberesrot = np.array([10, 255, 255])
    hsvBild = cv2.cvtColor(bild, cv2.COLOR_BGR2HSV)
    blMaskBild = cv2.inRange(hsvBild, unteresblau, oberesblau)
    grMaskBild = cv2.inRange(hsvBild, unteresgruen, oberesgruen)
    rttemp = cv2.inRange(hsvBild, unteresrot, oberesrot)
    grMaskBild = cv2.addWeighted(grMaskBild, 1, rttemp, 1, 0)
    #cv2.imshow("dasdj", grMaskBild)
    #cv2.imshow("streifenmask", blMaskBild)

    return hsvBild, blMaskBild, grMaskBild


def raender_detektion(blMask):
    raender = cv2.Canny(blMask, 100, 1200)

    return raender



def relevante_region_filter(raender, relevanz):
    hoehe, breite = raender.shape
    origmask =  np.zeros_like(raender)
    
    polygon = np.array([[(0, hoehe * relevanz), (breite, hoehe * relevanz), (breite, hoehe), (0, hoehe)]], np.int32)
    cv2.fillPoly(origmask, polygon, 255)


    cv2.fillPoly(origmask, polygon, 255)
    relevanteRaender = cv2.bitwise_and(raender, origmask)
    
    return relevanteRaender



def begrenzungs_detektion(relevanteRaender):
    minVotes = 7
    minLinienLaenge = 5
    maxLinenLuecke = 6 

    fahrbahn_linien = cv2.HoughLinesP(relevanteRaender, 1, np.pi/180, minVotes, np.array([]), minLinienLaenge, maxLinenLuecke)    

    return fahrbahn_linien


def begrenzungs_verbindung(breite, hoehe, fahrbahn_linien, relevanz):
    
    fahrstreifen = []
    
    if fahrbahn_linien is None:
        return fahrbahn_linien
        
    linker_teil = []
    rechter_teil = []
        
    grenze = 1/3
    linke_grenze = breite * (1 - grenze)
    rechte_grenze = breite * grenze
    
    for fahrbahn_linie in fahrbahn_linien:
        for x1, y1, x2, y2 in fahrbahn_linie:
            if x1 == x2:
                #vertikales Segment überspringen
                continue
            geradengleichung = np.polyfit((x1, x2), (y1, y2), 1)
            steigung = geradengleichung[0]
            y_achsenabschnitt = geradengleichung[1]
            
            if steigung < 0:
                if x1 < linke_grenze and x2 < linke_grenze and not(False):
                    linker_teil.append((steigung, y_achsenabschnitt))
            else:
                if x1 > rechte_grenze and x2 > rechte_grenze:
                    rechter_teil.append((steigung, y_achsenabschnitt))
                    
    
    if len(linker_teil) > 0:
        linker_teil_mittel = np.average(linker_teil, 0)
        fahrstreifen.append(punkte_erstellen(breite, hoehe, linker_teil_mittel, relevanz))
    
    if len(rechter_teil) > 0:
        rechter_teil_mittel = np.average(rechter_teil, 0)
        fahrstreifen.append(punkte_erstellen(breite, hoehe, rechter_teil_mittel, relevanz))
        
    return fahrstreifen



def punkte_erstellen(breite, hoehe, linie, relevanz):
    steigung, y_achsenabschnitt = linie
    
    y1 = hoehe
    y2 = int(y1 * relevanz)
    
    x1 = max( -breite, min(2 * breite, int((y1 - y_achsenabschnitt) / steigung)))
    x2 = max( -breite, min(2 * breite, int((y2 - y_achsenabschnitt) / steigung)))
    
    return [[x1, y1, x2, y2]]



def fahrtrichtung_linie(bild, fahrstreifen, relevanz):
    hoehe, breite, leer = bild.shape
    mitte = int(breite / 2)
    xVerschiebung = 0
    try:
        
        if(len(fahrstreifen) == 2):
            leer, leer, xLinks, leer = fahrstreifen[0][0]
            leer, leer, xRechts, leer = fahrstreifen[1][0]
            xVerschiebung = (xLinks + xRechts) / 2 - mitte
        elif(len(fahrstreifen) == 1):
            xt1, leer, xt2, leer = fahrstreifen[0][0]
            xVerschiebung = xt2 - xt1
    except:
        xVerschiebung = 0
        GPIO.output(StepPinForward, GPIO.HIGH)
        GPIO.output(StepPinnnForward, GPIO.HIGH)
        
    fahrtrichtung_linie = np.arange(4)
    
    fahrtrichtung_linie[1] = hoehe
    fahrtrichtung_linie[3] = int(hoehe * relevanz)
    fahrtrichtung_linie[0] = int(breite / 2)
    fahrtrichtung_linie[2] = int(mitte + xVerschiebung)
    
    return fahrtrichtung_linie
    
    
    
def winkel_berechnen(fahrtrichtung):
    
    gegenkathete = fahrtrichtung[1] - fahrtrichtung[3]
    ankathete = fahrtrichtung[0] - fahrtrichtung[2]
    
    if ankathete == 0:
        winkel = 90
    else:
        winkel = np.arctan(gegenkathete / ankathete) * (180 / np.pi)
    
    """
    cv2.putText(fahrstreifenBild, str(gegenkathete), (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(fahrstreifenBild, str(ankathete), (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    fahrstreifenBild = cv2.addWeighted(bild, 0.8, fahrstreifenBild, 1, 1)
    cv2.imshow("Fahrstreifenaaaaaa", fahrstreifenBild)
    """
    if winkel < 0:
        winkel = 180 + winkel
    
    winkel = 180 - winkel
    
    return winkel
     
    
    
def set_servo(winkel):
    if winkel < 0:
        winkel = 0
    if winkel > 180:
        winkel = 180
    #2.5 = 0% 7.5 = 50 12.5 = 100% (100% / 10 + 2.5)
    servoPwm = ((winkel + korrektur)  * (850 / 90)) + 650###################################################################################5
    
    pi.set_servo_pulsewidth(servopin, servoPwm)
    
    
    
def stabilisierung(winkel, winkel_jetzt, max_aenderung=5):
    aenderung = winkel - winkel_jetzt
    
    if abs(aenderung) > max_aenderung:
        winkel_stabilisiert = int(winkel_jetzt + (max_aenderung * aenderung / abs(aenderung)))
    else:
        winkel_stabilisiert = winkel
    
    return winkel_stabilisiert
    
    
    
def kreisDetektion(grMask, breite, bild):
    #cv2.imshow("orig", bild)
    test = np.zeros_like(bild)
    test = cv2.cvtColor(bild, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("grmask", grMask)
    grCanny = cv2.Canny(grMask, 600, 1200)
    #cv2.imshow("assdijasdwadj", grCanny)
    kreise = cv2.HoughCircles(grCanny, cv2.HOUGH_GRADIENT, 1, 150,
                               param1=1200, param2=25,
                               minRadius=35, maxRadius=200)
    
    if kreise is not None:
        kreise = np.uint16(np.around(kreise))
        for i in kreise[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(bild, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(bild, center, radius, (255, 0, 255), 8)
            
        
    cv2.imshow("kreise", bild)
    return kreise
    
    
    
    
        
while True:
    
    

    bild = stream.read()
    bild = cv2.resize(bild, (breite, hoehe), interpolation=cv2.INTER_AREA)
    bild = cv2.flip(bild, -1)

    forward(winkel_jetzt, bild, vI=v)

    hsv, blMask, grMask = konversionen(bild)


##Kreis-----------------------------------------------------------------
    
    graugruenfltr = cv2.medianBlur(grMask, 5)
    
    #graugruenfltr = cv2.GaussianBlur(grMask, ( 9, 9), 2, 2 );
    #cv2.imshow("gaussian blur", graugruenfltr)
    
    if kreisDetektion(graugruenfltr, breite, bild) is not None:
        ##Kreismaneuvration
        asd = 3
        if GPIO.input(ReedVorne) == HIGH:
            time.sleep(20)
        
    else:
        
##Kreis-----------------------------------------------------------------

        #cv2.imshow("HSV", hsv)
        #graugruenfltr = cv2.cvtColor(grMask, cv2.COLOR_BGR2GRAY)
        
        raender = raender_detektion(blMask)
        #cv2.imshow("Canny", raender) 


        relevanteRaender = relevante_region_filter(raender, relevanz)
        #cv2.imshow("Relevanter Teil", relevanteRaender)




        fahrbahn_linien = begrenzungs_detektion(relevanteRaender)
        
        """
        linienBild = np.copy(bild) * 0
        
        
        if fahrbahn_linien is not None:
            for linie in fahrbahn_linien:
                for x1, y1, x2, y2 in linie:
                    cv2.line(linienBild, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
        cv2.imshow("Linien", linienBild)
        """

        

        fahrstreifen = begrenzungs_verbindung(breite, hoehe, fahrbahn_linien, relevanz)

        
        fahrstreifenBild = np.zeros_like(bild)
        
        if fahrstreifen is not None:
            for fahrstreif in fahrstreifen:
                for x1, y1, x2, y2 in fahrstreif:
                    cv2.line(fahrstreifenBild, (x1, y1), (x2, y2), color=(0,255,0), thickness=10)
            cv2.putText(fahrstreifenBild, str(len(fahrstreifen)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            fahrstreifenBild = cv2.addWeighted(bild, 0.8, fahrstreifenBild, 1, 1)
            v = 20
        else:
            v = 0
            #cv2.imshow("Fahrstreifen", fahrstreifenBild)
        
        
        
        fahrtrichtung = fahrtrichtung_linie(bild, fahrstreifen, relevanz)
        
        winkel = winkel_berechnen(fahrtrichtung)
        
        winkel_jetzt = stabilisierung(winkel, winkel_jetzt)
        
        
        
        ######################################
        cv2.line(fahrstreifenBild, (fahrtrichtung[0], fahrtrichtung[1]), (fahrtrichtung[2], fahrtrichtung[3]), color=(0,0,255), thickness=10)    
        cv2.line(fahrstreifenBild, (int(breite / 2), hoehe), (int(int(breite / 2) - hoehe / 2 / math.tan((180 - winkel_jetzt) / 180.0 * math.pi)), int(hoehe / 2)), color=(255, 255, 255), thickness=4)
        """
        for x1, y1, x2, y2 in fahrtrichtung_linie:
            cv2.line(fahrstreifenBild, (x1, y1), (x2, y2), (0, 255, 0), 2)
        """
        


        timestamp = datetime.datetime.now()
        cv2.putText(fahrstreifenBild, timestamp.strftime(
            "%A %d %B %Y %I:%M:%S%p"), (10, fahrstreifenBild.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        
        cv2.putText(fahrstreifenBild, str(winkel), (150, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)    
        
        cv2.imshow("Endergebnis", fahrstreifenBild)
        
        ##########################################
        

    
        set_servo(winkel_jetzt)

        
        
        #Stream####################
        
      
        


    

    key = cv2.waitKey(1) & 0xFF
    
    
    #clear stream (bytes\/)
    #rawCapture.truncate(0)
    
    if key == ord("q"):
        cv2.destroyAllWindows()
        pi.set_servo_pulsewidth(servopin, 0)
        pi.stop()
        GPIO.cleanup()
        stream.stop()
        exit()
        break