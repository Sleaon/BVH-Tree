# Benchmark
## declare
* CPU 12th Gen Intel(R) Core(TM) i5-1240P 1.70 GHz
* Using 946 rectangles, perform separate tests for build time, containment relationship calculation time, and nearest neighbor relationship calculation time. Find 1,000 point at random.
## conclusion
| version | thread num | build cost (us) | contain one cost (us) | contain all cost (us) | nearst cost (us) | top-10 neighbor (us) |
| :------:| :---------:| :--------------:|:---------------------:|:-----------------------:|-------------------:|:---------------------:|
| Brute force search | 1 | 7478 | 240119 | 348138 | 914983 | null |
| BVH-Tree | 1 | 9022 | 598 | 740 | 17201 | 25730 | 