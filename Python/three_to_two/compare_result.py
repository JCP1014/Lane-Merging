def read_result(case):
    fp = open('result_' + case + '.txt', 'r')
    lines = fp.read().split('\n')[:-1]
    caseNum = len(lines) / 2
    dp_total_win = 0
    fcfg_total_win = 0
    # dp_wait_win = 0
    # fcfg_wait_win = 0
    total_tie = 0
    # wait_tie = 0
    dp_total_sum = 0
    fcfg_total_sum = 0
    # dp_wait_sum = 0
    # fcfg_wait_sum = 0
    computeTime_sum = 0

    i = 0
    while i < len(lines):
        dp_data = lines[i].split(' ')
        fcfg_data = lines[i+1].split(' ')
        # print(dp_data[2], fcfg_data[2])
        # print(dp_data[3], fcfg_data[3])
        dp_total_sum += float(dp_data[1])
        fcfg_total_sum += float(fcfg_data[1])
        # dp_wait_sum += float(dp_data[3])
        # fcfg_wait_sum += float(fcfg_data[3])

        if float(dp_data[1]) < float(fcfg_data[1]):
            dp_total_win += 1
            # print('dp_total_win')
        elif float(dp_data[1]) > float(fcfg_data[1]):
            fcfg_total_win += 1
            # print('fcfg_total_win')
        else:
            total_tie += 1
            # print('total_tie')
        
        # if float(dp_data[3]) < float(fcfg_data[3]):
        #     dp_wait_win += 1
        #     # print('dp_wait_win')
        # elif float(dp_data[3]) > float(fcfg_data[3]):
        #     fcfg_wait_win += 1
        #     # print('fcfg_wait_win')
        # else:
        #     wait_tie += 1
        #     # print('wait_tie')

        computeTime_sum += float(dp_data[2])
        i += 2

    print('case:', case)
    print('dp_total_win:', dp_total_win, '(' + str(dp_total_win/caseNum*100) + '%)')
    print('fcfg_total_win:', fcfg_total_win, '(' + str(fcfg_total_win/caseNum*100) + '%)')
    print('total_tie:', total_tie, '(' + str(total_tie/caseNum*100) + '%)')
    # print('dp_wait_win:', dp_wait_win, '(' + str(dp_wait_win/caseNum*100) + '%)')
    # print('fcfg_wait_win:', fcfg_wait_win, '(' + str(fcfg_wait_win/caseNum*100) + '%)')
    # print('wait_tie:', wait_tie, '(' + str(wait_tie/caseNum*100) + '%)')
    print('dp_total_avg:', str(dp_total_sum/caseNum))
    print('fcfg_total_avg:', str(fcfg_total_sum/caseNum))
    # print('dp_wait_avg:', str(dp_wait_sum/caseNum))
    # print('fcfg_wait_avg:', str(fcfg_wait_sum/caseNum))
    print('dp_computeTime_avg:', str(computeTime_sum/caseNum))
    print('\n')
    

def main():
    read_result('01')
    read_result('02')
    read_result('03')
    read_result('04')
    read_result('05')
    read_result('20')
    read_result('40')
    read_result('60')
    read_result('80')
    read_result('100')
    read_result('w10')
    read_result('w12')
    read_result('w14')
    read_result('w16')
    read_result('w18')
    read_result('w20')
    read_result('w22')
    read_result('w24')
    read_result('w26')
    read_result('w28')
    read_result('w30')


if __name__ == '__main__':
    main()
