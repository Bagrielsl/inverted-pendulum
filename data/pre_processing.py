import numpy as np
import pandas as pd

def test_soltar():
    with open('test_soltar.txt', 'r') as file:
        code = file.read()
    t = []
    r = []
    p = []
    for line in code.split('\n')[1:]:
        sline = line.split()
        if len(sline) == 6:        
            t.append(
                float(sline[1])
            )
            r.append(
                float(sline[3])
            )
            p.append(
                float(sline[5])
            )
        else:
            print('error:',line)
    t = np.array(t)
    r = np.array(r)
    p = np.array(p)
    t = t - t[0]
    with open('test_soltar_out.csv', 'w') as file:
        file.write('t,r,p\n')
        for i in range(len(t)):
            file.write(f'{t[i]},{r[i]},{p[i]}\n')
    print('wrote test_soltar_out.csv')
    data = pd.read_csv('test_soltar_out.csv')
    data = data[data['t'] > 46300]
    data['p'] = -data['p'] + data['p'].max()
    data['t'] = (data['t'] - data['t'].iloc[0]) / 1000
    data.to_csv('test_1.csv', index=False)
    print('wrote test_1.csv')
    
def test_empurrar():
    with open('test_empurrar.txt', 'r') as file:
        code = file.read()
    part1 = False
    part2 = False
    t = []
    r = []
    p = []
    f = []
    for line in code.split('\n')[1:]:
        if 'Vai comecar!!!!!!' in line:
            part1 = True
        if 'Acabou de se movimentar!!!!!!!!' in line:
            part2 = True
        sline = line.split()
        try:
            new_t, new_r, new_p = sline[1], sline[3], sline[5]
            new_t, new_r, new_p = float(new_t), float(new_r), float(new_p)
            if part1:
                for a,b in zip([t, r, p], [new_t, new_r, new_p]):
                    a.append(b)
                f.append( 0.0 if part2 else 1.0)
        except:
            print('error:',line)
    t = np.array(t)
    r = np.array(r)
    p = np.array(p)
    f = np.array(f)
    t = t - t[0]
    print(len(t), len(r), len(p), len(f))
    with open('test_empurrar_out.csv', 'w') as file:
        file.write('t,r,p,f\n')
        for i in range(len(t)):
            file.write(f'{t[i]},{r[i]},{p[i]},{f[i]}\n')
    print('wrote test_empurrar_out.csv')
    data = pd.read_csv('test_empurrar_out.csv')
    data = data[data['t'] < 30000]
    data['p'] = -data['p'] + data['p'].max()
    data['t'] = (data['t'] - data['t'].iloc[0]) / 1000
    data.to_csv('test_2.csv', index=False)
    print('wrote test_2.csv')

test_soltar()
test_empurrar()