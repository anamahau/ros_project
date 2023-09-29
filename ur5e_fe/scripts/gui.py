import PySimpleGUI as sg

#sg.theme_previewer()

sg.theme('DarkAmber')

izdelki = ['mleko', 'sok']
seznam_izdelkov = ""
for i in izdelki:
    if len(seznam_izdelkov) == 0:
        seznam_izdelkov += i
    else:
        seznam_izdelkov += ", " + i

layout = [[sg.Text('Izdelki, ki so na voljo: ' + seznam_izdelkov)],
          [sg.Text('Izberi izdelek'), sg.InputText()],
          [sg.Text('Vnesi količino'), sg.InputText()],
          [sg.Button('Oddaj narocilo'), sg.Button('Preklici narocilo')]]

window = sg.Window('Oddaja narocila', layout)

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Preklici narocilo':
        break
    elif event == 'Oddaj narocilo':
        izdelek = values[0]
        kolicina = values[1]
        izdelek_OK = False
        kolicina_OK = True
        if izdelek in izdelki:
            izdelek_OK = True
        else:
            izdelek_OK = False
            print('Izbrani izdelek zal ne obstaja.')
        if not kolicina.isnumeric():
            kolicina_OK = False
            print('Prosim, zapisite kolicino s stevilko.')
        else:
            kolicina_OK = True
        if izdelek_OK and kolicina_OK:
            print('Narocilo oddano. Podatki o narocilu: ' + str(kolicina) + 'x ' + str(izdelek))
            break

window.close()