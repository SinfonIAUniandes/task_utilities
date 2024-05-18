from office365.runtime.auth.client_credential import ClientCredential
from office365.sharepoint.client_context import ClientContext
import pandas as pd

# Configura tus credenciales
client_id = 'your_client_id'
client_secret = 'your_client_secret'
site_url = 'https://uniandes-my.sharepoint.com'
relative_url = '/personal/f_hernandezt_uniandes_edu_co/Documents/Inscripción de Asistentes al Evento de ZEISS.xlsx'

# Autenticación
ctx = ClientContext(site_url).with_credentials(ClientCredential(client_id, client_secret))

# Leer el archivo Excel desde SharePoint
response = ctx.web.get_file_by_server_relative_url(relative_url).download()

# Guardar el archivo temporalmente
with open("event_data.xlsx", "wb") as local_file:
    local_file.write(response.content)

# Cargar el archivo Excel en un DataFrame de pandas
df = pd.read_excel("event_data.xlsx")
print(df.head())
