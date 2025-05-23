import cv2
import numpy as np

def compute_pose_cv(frame):
    # load object detection model
    # we'll start with bin detection onlyt -- if bin is detected, conf = 0.5 (arbitrary nonzero param)
    # else conf = 0
    # then do distance estimation to detected object
    pass


# sample code for: https://huggingface.co/hustvl/yolos-tiny
from transformers import YolosImageProcessor, YolosForObjectDetection
from PIL import Image
import torch
import os
import sys
import requests

#url = "http://images.cocodataset.org/val2017/000000039769.jpg"
#image = Image.open(requests.get(url, stream=True).raw)
image = Image.open(os.path.join(sys.path[0], "data/sample_field.jpg"))

model = YolosForObjectDetection.from_pretrained('hustvl/yolos-tiny')
image_processor = YolosImageProcessor.from_pretrained("hustvl/yolos-tiny")

inputs = image_processor(images=image, return_tensors="pt")
outputs = model(**inputs)

# model predicts bounding boxes and corresponding COCO classes
logits = outputs.logits
bboxes = outputs.pred_boxes


# print results
target_sizes = torch.tensor([image.size[::-1]])
results = image_processor.post_process_object_detection(outputs, threshold=0.9, target_sizes=target_sizes)[0]
for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
    box = [round(i, 2) for i in box.tolist()]
    print(
        f"Detected {model.config.id2label[label.item()]} with confidence "
        f"{round(score.item(), 3)} at location {box}"
    )