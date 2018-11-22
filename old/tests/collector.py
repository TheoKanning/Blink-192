from blink192.collector import ContinuousCollector

collector = ContinuousCollector()
collector.start()

raw_input("Testing collector. Press enter to stop")
collector.stop()