def read_case(steps):
    fp = open('result_' + str(steps) + '.txt', 'r')
    lines = fp.read().split('\n')[:-1]
    caseNum = len(lines) / 2
    vehNum_list = []
    dp_total_win = 0
    fafg_total_win = 0
    dp_wait_win = 0
    fafg_wait_win = 0
    total_tie = 0
    wait_tie = 0

    i = 0
    while i < len(lines):
        dp_data = lines[i].split(' ')
        fafg_data = lines[i+1].split(' ')
        if int(dp_data[1]) == int(fafg_data[1]):
            vehNum_list.append(int(dp_data[1]))
        else:
            print('bug')
            return
        # print(dp_data[2], fafg_data[2])
        # print(dp_data[3], fafg_data[3])
        if float(dp_data[2]) < float(fafg_data[2]):
            dp_total_win += 1
            # print('dp_total_win')
        elif float(dp_data[2]) > float(fafg_data[2]):
            fafg_total_win += 1
            # print('fafg_total_win')
        else:
            total_tie += 1
            # print('total_tie')
        
        if float(dp_data[3]) < float(fafg_data[3]):
            dp_wait_win += 1
            # print('dp_wait_win')
        elif float(dp_data[3]) > float(fafg_data[3]):
            fafg_wait_win += 1
            # print('fafg_wait_win')
        else:
            wait_tie += 1
            # print('wait_tie')
        i += 2

    print('time steps:', steps)
    print('vehNum range:', '[' + str(min(vehNum_list)) + ', ' + str(max(vehNum_list)) + ']')
    print('dp_total_win:', dp_total_win, '(' + str(dp_total_win/caseNum*100) + '%)')
    print('fafg_total_win:', fafg_total_win, '(' + str(fafg_total_win/caseNum*100) + '%)')
    print('total_tie:', total_tie, '(' + str(total_tie/caseNum*100) + '%)')
    print('dp_wait_win:', dp_wait_win, '(' + str(dp_wait_win/caseNum*100) + '%)')
    print('fafg_wait_win:', fafg_wait_win, '(' + str(fafg_wait_win/caseNum*100) + '%)')
    print('wait_tie:', wait_tie, '(' + str(wait_tie/caseNum*100) + '%)')
    print('\n')
    

def main():
    read_case(60)
    read_case(300)
    read_case(600)
    read_case(1800)
    read_case(3600)

if __name__ == '__main__':
    main()