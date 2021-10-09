import sys
import json
import numpy as np


def main():
    path = sys.argv[1]

    topics = {}
    with open(path) as file:
        bag = json.load(file)
        for timestamp, topic, message in bag:
            if topic not in topics:
                topics[topic] = []
                print("\n-----\nFound topic %s: \n%s" % (topic, message))
            topics[topic].append(timestamp)
    print("Topic rates:")
    for topic, timestamps in topics.items():
        rate = 1.0 / np.average(np.diff(timestamps))
        print("\t%s: \t%0.2f Hz" % (topic, rate))


if __name__ == "__main__":
    main()
