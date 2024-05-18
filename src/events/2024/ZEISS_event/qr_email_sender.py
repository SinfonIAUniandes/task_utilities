import qrcode
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

# Función para crear un código QR
def create_qr(data, file_path):
    qr = qrcode.QRCode(version=1, box_size=10, border=5)
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image(fill='black', back_color='white')
    img.save(file_path)

# Función para enviar un correo electrónico
def send_email(to_email, subject, body, img_path):
    from_email = "your_email@example.com"
    from_password = "your_password"
    
    msg = MIMEMultipart()
    msg['From'] = from_email
    msg['To'] = to_email
    msg['Subject'] = subject
    
    msg.attach(MIMEText(body, 'plain'))
    
    with open(img_path, 'rb') as f:
        img = MIMEImage(f.read())
        img.add_header('Content-Disposition', 'attachment', filename='qr_code.png')
        msg.attach(img)
    
    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()
    server.login(from_email, from_password)
    text = msg.as_string()
    server.sendmail(from_email, to_email, text)
    server.quit()

# Datos del invitado
guest_name = "Juan Perez"
email = "juan.perez@example.com"
company_name = "Optica ABC"
custom_message = f"Hola {guest_name}, bienvenido al evento de ZEISS, espero lo disfrutes mucho!"

# Crear QR personalizado
qr_data = {"guest_name": guest_name, "email": email, "company_name": company_name}
qr_file_path = "qr_code.png"
create_qr(str(qr_data), qr_file_path)

# Enviar correo con QR
subject = "Tu código QR para el evento de ZEISS"
body = f"Hola {guest_name}, adjunto encontrarás tu código QR para el evento."
send_email(email, subject, body, qr_file_path)
