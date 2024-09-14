import pandas as pd
import datetime

# Definir el rango de fechas para el próximo semestre
start_date = datetime.date(2024, 7, 29)  # Inicio del semestre
num_weeks = 20  # Duración del semestre en semanas

# Crear la lista de semanas y sus fechas de inicio y fin
weeks_data = []

for i in range(num_weeks):
    week_start = start_date + datetime.timedelta(weeks=i)
    week_end = week_start + datetime.timedelta(days=6)
    weeks_data.append([
        f'Semana {i+1}',
        week_start,
        week_end,
        '',  # Espacio para rellenar tareas realizadas
        '',  # Espacio para rellenar resultados obtenidos
        '',  # Espacio para rellenar problemas encontrados
        '',  # Espacio para rellenar soluciones propuestas
        ''   # Espacio para rellenar comentarios adicionales
    ])

# Crear un DataFrame con los datos
bitacora_columns = [
    'Semana',
    'Fecha de inicio',
    'Fecha de fin',
    'Tareas realizadas',
    'Resultados obtenidos',
    'Problemas encontrados',
    'Soluciones propuestas',
    'Comentarios adicionales'
]

bitacora_df = pd.DataFrame(weeks_data, columns=bitacora_columns)

# Guardar el DataFrame en una nueva hoja en el archivo Excel
file_path = 'C:/Users/nicolas/Desktop/Memoria/gantt_ex.xlsx'  # Cambia esto a la ruta de tu archivo

with pd.ExcelWriter(file_path, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
    bitacora_df.to_excel(writer, sheet_name='Bitacora_Semanal', index=False)

print(f'Bitácora semanal creada y guardada en {file_path}')
