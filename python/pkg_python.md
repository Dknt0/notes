# 参数管理 argprase

```py
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--param1', help='Parameter for test', default='Hello world!')
args = parser.parse_args()

print(args.param1)
```


