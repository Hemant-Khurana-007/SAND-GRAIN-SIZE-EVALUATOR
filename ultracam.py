import serial
import cv2
import math
import time
import numpy as np
from scipy import ndimage
from skimage import measure
import pandas as pd


import cv2
import math
import numpy as np
from scipy import ndimage
from skimage import measure
import pandas as pd
captured = False
def ml(rimg, area_in_mm):
    global lat,lon
    # ---------- STEP 1: LOAD IMAGE ----------
    img_color = cv2.imread(rimg)
    if img_color is None:
        raise FileNotFoundError(f"Image not found: {rimg}")

    height, width = img_color.shape[:2]
    area_pix = height * width

    # scaling factor
    pixels_per_mm = math.sqrt(area_pix / area_in_mm)

    # read grayscale for processing
    img = cv2.imread(rimg, 0)

    # ---------- STEP 2: THRESHOLD ----------
    _, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # ---------- STEP 3: CLEANUP ----------
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(thresh, kernel, iterations=1)
    dilated = cv2.dilate(eroded, kernel, iterations=1)
    mask = dilated == 255

    # ---------- STEP 4: LABEL OBJECTS ----------
    s = [[0,1,0],
     [1,1,1],
     [0,1,0]]   # 4-connectivity

    #s = [[1,1,1],[1,1,1],[1,1,1]]   # 8-connectivity
    labeled_mask, num_labels = ndimage.label(mask, structure=s)
    clusters = measure.regionprops(labeled_mask)

    # ---------- STEP 5: MEASURE GRAINS ----------
    min_size_mm = 0.05#0.05  # filter very small particles
    grain_sizes = []
    for prop in clusters:
        equiv_diam_px = prop.equivalent_diameter   # in pixels
        equiv_diam_mm = equiv_diam_px / pixels_per_mm
        if equiv_diam_mm >= min_size_mm:
            grain_sizes.append(equiv_diam_mm)

    grain_sizes = np.array(grain_sizes)
    if len(grain_sizes) == 0:
        raise ValueError("No valid grains detected.")

    # ---------- STEP 6: COMPUTE DISTRIBUTION ----------
    D10 = np.percentile(grain_sizes, 10)
    D16 = np.percentile(grain_sizes, 16)
    D25 = np.percentile(grain_sizes, 25)
    D50 = np.percentile(grain_sizes, 50)
    D65 = np.percentile(grain_sizes, 65)
    D75 = np.percentile(grain_sizes, 75)
    D84 = np.percentile(grain_sizes, 84)
    D90 = np.percentile(grain_sizes, 90)

    Dmean = np.mean(grain_sizes)
    Dmed = (D16 + D50 + D84) / 3

    # ---------- CLASSIFICATION ----------
    def classify_sediment(dmed):
            if dmed < 0.063:
                return "Silt/Clay"
            elif dmed < 0.2:
                return "fine sand"
            elif dmed < 0.63:
                return "Medium Sand"
            elif dmed < 2.0:
                return "Coarse Sand"
            else:
                return "Gravel"

    sediment_type = classify_sediment(Dmed)

    # ---------- SAVE SUMMARY ----------
    summary = {
        "Number_of_Grains": len(grain_sizes),
        "D10": D10, "D16": D16, "D25": D25, "D50": D50,
        "D65": D65, "D75": D75, "D84": D84, "D90": D90,
        "Dmean": Dmean, "Dmed": Dmed,
        "Sediment_Type": sediment_type,
        "LAT":lat,
        "LON":lon
    }

    df = pd.DataFrame([summary])
    #df.to_csv("grain_summary.csv", index=False)
    df.to_csv("grain_summary.csv", mode="a",header=not pd.io.common.file_exists("grain_summary.csv"), index= False)
    # Print summary
    print("\n===== Grain Size Analysis Results =====")
    for k, v in summary.items():
        if isinstance(v, float):
            print(f"{k}: {v:.3f} mm")
        else:
            print(f"{k}: {v}")
    print("Results saved to 'grain_summary.csv'")
    return [summary["Number_of_Grains"],summary["Dmed"],summary["Sediment_Type"]]

# ---------- CAMERA & SERIAL LOOP ----------
cap = cv2.VideoCapture(0)
port = '/dev/ttyACM0'
baudrate = 9600
ser = serial.Serial(port, baudrate, timeout=1)

print(f"Listening on {port} at {baudrate} baud...")

if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame from camera")
            break
        
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            try:
                parts = line.split('|')
                mean  = int(parts[2][6:].strip())
                roll  = float(parts[3][6:].strip())
                pitch = float(parts[4][7:].strip())
                lat   = float(parts[5][5:].strip())
                lon   = float(parts[6][5:].strip())
            except (IndexError, ValueError):
                continue
        
                  

            if (9 < mean < 11) and (-2< pitch <2)and ((178<roll<182)or(-182<roll<-178)) and not captured:
                captured = True
                # Safe filename without spaces or colons
                filename = time.strftime("%Y%m%d_%H%M%S")+".jpg"
                cv2.imwrite(filename, frame)
                print(f"Frame saved as {filename}")
                model = ml(filename,2925)
                no = model[0]
                dia = model[1]
                typ = model[2]
                cv2.putText(frame, f"NO OF GRAINS: {no}",  (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
                cv2.putText(frame, f"DIAMETER: {dia}",  (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
                cv2.putText(frame, f"TYPE: {typ}", (10,110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
                cv2.imshow("Captured Frame", frame)
                while True:
                    k = cv2.waitKey(0) & 0xFF
                    if k == ord('e') or k == ord('E'):
                        captured = False
                        break
                ser.reset_input_buffer()   
                cv2.destroyWindow("Captured Frame")
                
        cv2.putText(frame, f"Mean: {mean}",  (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(frame, f"Roll: {roll}",  (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        cv2.putText(frame, f"Pitch: {pitch}", (10,110),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

        cv2.imshow('Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    ser.close()
    cap.release()
    cv2.destroyAllWindows()
