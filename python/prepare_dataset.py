import os
import shutil
import random
from pathlib import Path

RAW_IMAGES = Path("raw_images")
LABELS_DIR = Path("makesense_labels")
DATASET = Path("dataset")

CLASS_NAME = "target_object"
TRAIN_RATIO = 0.8

image_exts = [".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"]

# Remove old dataset if it exists
if DATASET.exists():
    shutil.rmtree(DATASET)

# Create YOLO folder structure
for split in ["train", "val"]:
    (DATASET / "images" / split).mkdir(parents=True, exist_ok=True)
    (DATASET / "labels" / split).mkdir(parents=True, exist_ok=True)

# Collect images
images = []
for ext in image_exts:
    images.extend(RAW_IMAGES.glob(f"*{ext}"))

images = sorted(images)

# Collect label files recursively
label_files = {}
for txt in LABELS_DIR.rglob("*.txt"):
    if txt.name.lower() in ["classes.txt", "labels.txt"]:
        continue
    label_files[txt.stem] = txt

pairs = []
missing_labels = []

for img in images:
    label = label_files.get(img.stem)
    if label and label.exists():
        pairs.append((img, label))
    else:
        missing_labels.append(img.name)

random.seed(42)
random.shuffle(pairs)

train_count = int(len(pairs) * TRAIN_RATIO)
train_pairs = pairs[:train_count]
val_pairs = pairs[train_count:]

def copy_pairs(pairs, split):
    for img, label in pairs:
        shutil.copy(img, DATASET / "images" / split / img.name)
        shutil.copy(label, DATASET / "labels" / split / f"{img.stem}.txt")

copy_pairs(train_pairs, "train")
copy_pairs(val_pairs, "val")

data_yaml = f"""path: dataset
train: images/train
val: images/val

names:
  0: {CLASS_NAME}
"""

(DATASET / "data.yaml").write_text(data_yaml)

print("Dataset prepared!")
print(f"Total images found: {len(images)}")
print(f"Total labeled image-label pairs: {len(pairs)}")
print(f"Train images: {len(train_pairs)}")
print(f"Validation images: {len(val_pairs)}")

if missing_labels:
    print(f"Warning: {len(missing_labels)} images had no matching label and were skipped.")
    print("First missing examples:", missing_labels[:10])

print("\ndata.yaml:")
print(data_yaml)
