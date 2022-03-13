import git
import os
import argparse
import shutil


parser = argparse.ArgumentParser()
PATH = os.getcwd()
parser.add_argument("--weights", metavar="weights", type=str)
parser.add_argument("--conf-thres", metavar="conf-thres", type=float, default=.3)
parser.add_argument("--workspace", metavar="workspace", type=int, default=1000)
args = parser.parse_args()

try:

	git.Git().clone("https://github.com/ultralytics/yolov5")
	os.chdir("yolov5")
	os.system("python3 export.py --weights {} --conf-thres {} --include onnx".format(PATH + "/" + args.weights, args.conf_thres))
	prefix = PATH + "/" + args.weights.split(".")[0]
	onnx = prefix + ".onnx"
	engine = prefix + ".engine"
	#os.chdir()
	os.system("trtexec --onnx={} --explicitBatch --saveEngine={} --workspace={} --fp16".format(onnx, engine, args.workspace))


except Exception as e:
	print(e)

shutil.rmtree(PATH + "/yolov5", ignore_errors=True)
