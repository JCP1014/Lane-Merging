def count_waiting_time():
    fp = open('tripinfo_dp.xml', 'r')
    info = fp.read().split('tripinfo_file.xsd">\n')[1].split('\n')[:-2]
    num_dp = len(info)
    delay_dp = 0
    end_dp = 0
    for i in info:
        delay_dp += float(i.split('waitingTime="')[1].split('"')[0])
        end_dp = float(i.split('arrival="')[1].split('"')[0])
    avgDelay_dp = delay_dp / num_dp
    fp.close()

    fp = open('tripinfo_fafg.xml', 'r')
    info = fp.read().split('tripinfo_file.xsd">\n')[1].split('\n')[:-2]
    num_fafg = len(info)
    delay_fafg = 0
    end_fafg = 0
    for i in info:
        delay_fafg += float(i.split('waitingTime="')[1].split('"')[0])
        end_fafg = float(i.split('arrival="')[1].split('"')[0])
    avgDelay_fafg = delay_fafg / num_fafg
    fp.close()

    print('dp:', num_dp, end_dp, avgDelay_dp)
    print('fafg:', num_fafg, end_fafg, avgDelay_fafg)
    

def main():
    count_waiting_time()

if __name__ == '__main__':
    main()