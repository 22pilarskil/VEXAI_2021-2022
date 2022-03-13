import json
from pathlib import Path
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("--ignore", metavar="ignore", nargs='+', type=str)
parser.add_argument("--in_json", metavar="in_json", type=str)
parser.add_argument("--out_json", metavar="out_json", type=str)
args = parser.parse_args()


args.ignore = ["Balance", "Bot"]

with open(Path(args.in_json)) as f:
	data = json.load(f)
	for i in range(len(data)):
		repeat = True
		while(repeat):
			repeat = False
			for x in data[i]['Label']['objects']:
				if x['title'] in ignore:
					repeat = True
					data[i]['Label']['objects'].remove(x)


	with open(args.out_json, 'w+') as z:
		json.dump(data, z)
