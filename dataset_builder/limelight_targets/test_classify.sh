imagenet --model=classify_model/limelight-classify.onnx --input_blob=input_0 --output_blob=output_0 --labels=classify_dataset/labels.txt classify_dataset/val/target_1/00_original_target_1-00017.jpg output.jpg
xdg-open output.jpg
