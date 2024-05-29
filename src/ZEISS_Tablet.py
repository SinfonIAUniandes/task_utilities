import rospy
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from robot_toolkit_msgs.msg import speech_recognition_status_msg

rospy.init_node("tablet_node")
hot_word_publisher = rospy.Publisher("/pytoolkit/ALSpeechRecognition/status",speech_recognition_status_msg,queue_size=10)
html = """
        <!DOCTYPE html>
        <html lang="es">
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <title>Interfaz de Opciones</title>
            <link rel="stylesheet" type="text/css" href="/static/styles.css">
        </head>
        <body>
            <div class="container">
                <div class="logo-container">
                    <img src="/static/logo.png" alt="Logo" class="logo">
                </div>
                <div class="button-container">
                    <form action="/action" method="post">
                        <button type="submit" name="action" value="QR" class="option-button" style="background-color: #ffffff;">QR</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="detente" class="option-button" style="background-color: #FFA500;">Detente</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="heynova" class="option-button" style="background-color: #FFD700;">Hey nova</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="baile" class="option-button" style="background-color: #ADFF2F;">Baile</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="asereje" class="option-button" style="background-color: #32CD32;">Asereje</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="pose" class="option-button" style="background-color: #00FA9A;">Pose</button>
                    </form>
                    
                    <form action="/action" method="post">
                        <button type="submit" name="action" value="fact" class="option-button" style="background-color: #BA55F8;">Fact</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="musculos" class="option-button" style="background-color: #00CED1;">Musculos</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="besos" class="option-button" style="background-color: #1E90FF;">Besos</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="guitarra" class="option-button" style="background-color: #8A2BE2;">Guitarra</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="cumpleaños" class="option-button" style="background-color: #9932CC;">Cumpleaños</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="corazon" class="option-button" style="background-color: #C71585;">Corazon</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="helicoptero" class="option-button" style="background-color: #FF4500;">Helicoptero</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="zombi" class="option-button" style="background-color: #DC143C;">Zombi</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="carro" class="option-button" style="background-color: #FF69B4;">Carro</button>
                    </form>

                    <form action="/action" method="post">
                        <button type="submit" name="action" value="gracias" class="option-button" style="background-color: #BA55D3;">Gracias</button>
                    </form>
                </div>
            </div>
            
        <script>
            document.addEventListener('DOMContentLoaded', function() {
                const form = document.getElementById('action-form');
                const actionInput = document.getElementById('action-input');

                form.addEventListener('click', function(event) {
                    if (event.target.tagName === 'BUTTON') {
                        const action = event.target.value;
                        actionInput.value = action;
                        form.submit();
                    }
                });
            });
        </script>
        </body>
        </html>
    """

app = FastAPI()

app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/", response_class=HTMLResponse)
@app.head("/", response_class=HTMLResponse)
@app.post("/", response_class=HTMLResponse)
async def main_page():
    return html

@app.post("/action", response_class=HTMLResponse)
@app.head("/action", response_class=HTMLResponse)
async def handle_action(request: Request):
    form_data = await request.form()
    action = form_data.get('action')
    print(action)
    hot_word_publisher.publish(action)
    return html

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)