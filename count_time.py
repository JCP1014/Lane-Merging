def main():
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

    fp = open('tripinfo_fcfg.xml', 'r')
    info = fp.read().split('tripinfo_file.xsd">\n')[1].split('\n')[:-2]
    num_fcfg = len(info)
    delay_fcfg = 0
    end_fcfg = 0
    for i in info:
        delay_fcfg += float(i.split('waitingTime="')[1].split('"')[0])
        end_fcfg = float(i.split('arrival="')[1].split('"')[0])
    avgDelay_fcfg = delay_fcfg / num_fcfg
    fp.close()

    print('dp:', num_dp, end_dp, avgDelay_dp)
    print('fcfg:', num_fcfg, end_fcfg, avgDelay_fcfg)

if __name__ == '__main__':
    main()