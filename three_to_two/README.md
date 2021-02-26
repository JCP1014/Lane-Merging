# 3-to-2 Lane Merging
## three_to_two.py
* namedtuple "Sol"
```
# The tuple of solution stored in each "square" in the table
Sol = namedtuple("Sol", "time table lane")  # Each solution has three fields.
                                            # "time" means the scheduled entering time of the last vehicle
                                            # "table" means which table the optimal solution is from
                                            # "lane" means which lane the last vehicle go to
```
* ```generate_traffic_v1(timeStep, alpha, beta, gamma, pA, pB, pC)```  
  * timeStep: 時間精度(秒)  
  * alpha: Lane A 上要產生幾台車  
  * beta: Lane B 上要產生幾台車   
  * gamma: Lane C 上要產生幾台車   
  * pA: Lane A 上 Poisson distribution 的 λ 值  
  * pB: Lane B 上 Poisson distribution 的 λ 值  
  * pC: Lane C 上 Poisson distribution 的 λ 值  
  * return a, b, c: 分別是 Lane A/B/C 上每台車的 earliest arrival time  

* ```generate_traffic_v2(timeStep, alpha, beta, gamma, p)```  
  * timeStep: 時間精度(秒)  
  * alpha: Lane A 上要產生幾台車  
  * beta: Lane B 上要產生幾台車   
  * gamma: Lane C 上要產生幾台車   
  * p: Poisson distribution 的 λ 值  
  * ```return a, b, c```: 分別是 Lane A/B/C 上每台車的 earliest arrival time  

* ```get_obj(sol)```
  * sol: a namedtuple of solution, 即 table 的每一格裡存的東西  
  * ```return max(sol.time)```: 因為 Sol 的 time field 存的是一組包含兩個值的tuple(x, y)，其中 x 代表去 Lane X 的最後一台車通過路口的時間，y 代表去 Lane Y 的最後一台車通過路口的時間，兩個值之間較大的才代表所有車都通過路口的時間，也就是我們要 minimize 的 objective value  

* ```multiDim_dp(a, b, c, W_same, W_diff)```  
  * a: 一個 list, 裡面是 Lane A 上每台車的 earliest arrival time  
  * b: 一個 list, 裡面是 Lane B 上每台車的 earliest arrival time  
  * c: 一個 list, 裡面是 Lane C 上每台車的 earliest arrival time  
  * W_same: 連續兩台通過路口的車若來自同一條 Lane, 至少需間隔的時間 (秒)  
  * W_diff: 連續兩台通過路口的車若來自不同條 Lane, 至少需間隔的時間 (秒)  
  * ```return T_last, computeTime```:
    * T_last: 最後一台車通過路口的時間 （最後一台車通過就代表所有車都通過）
    * computeTime: 計算出最佳排程的所需的運算時間


