def read_result(steps):
    fp = open('result_' + str(steps) + '.txt', 'r')
    lines = fp.read().split('\n')[:-1]
    caseNum = len(lines) / 2
    vehNum_list = []
    dp_total_win = 0
    fcfg_total_win = 0
    dp_wait_win = 0
    fcfg_wait_win = 0
    total_tie = 0
    wait_tie = 0

    i = 0
    while i < len(lines):
        dp_data = lines[i].split(' ')
        fcfg_data = lines[i+1].split(' ')
        if int(dp_data[1]) == int(fcfg_data[1]):
            vehNum_list.append(int(dp_data[1]))
        else:
            print('bug')
            return
        # print(dp_data[2], fcfg_data[2])
        # print(dp_data[3], fcfg_data[3])
        if float(dp_data[2]) < float(fcfg_data[2]):
            dp_total_win += 1
            # print('dp_total_win')
        elif float(dp_data[2]) > float(fcfg_data[2]):
            fcfg_total_win += 1
            # print('fcfg_total_win')
        else:
            total_tie += 1
            # print('total_tie')
        
        if float(dp_data[3]) < float(fcfg_data[3]):
            dp_wait_win += 1
            # print('dp_wait_win')
        elif float(dp_data[3]) > float(fcfg_data[3]):
            fcfg_wait_win += 1
            # print('fcfg_wait_win')
        else:
            wait_tie += 1
            # print('wait_tie')
        i += 2

    print('time steps:', steps)
    print('vehNum range:', '[' + str(min(vehNum_list)) + ', ' + str(max(vehNum_list)) + ']')
    print('dp_total_win:', dp_total_win, '(' + str(dp_total_win/caseNum*100) + '%)')
    print('fcfg_total_win:', fcfg_total_win, '(' + str(fcfg_total_win/caseNum*100) + '%)')
    print('total_tie:', total_tie, '(' + str(total_tie/caseNum*100) + '%)')
    print('dp_wait_win:', dp_wait_win, '(' + str(dp_wait_win/caseNum*100) + '%)')
    print('fcfg_wait_win:', fcfg_wait_win, '(' + str(fcfg_wait_win/caseNum*100) + '%)')
    print('wait_tie:', wait_tie, '(' + str(wait_tie/caseNum*100) + '%)')
    print('\n')
    

def main():
    read_result(60)
    read_result(600)
    read_result(1800)
    read_result(3600)

if __name__ == '__main__':
    main()
