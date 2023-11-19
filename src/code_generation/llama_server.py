from llama import Llama
from flask import request
from flask import Flask
import torch.distributed as dist
import time

LLAMA2 = {
            "path": './llama/llama-2-13b-chat',
            "tokenizer": './llama/tokenizer.model'
        }

app = Flask(__name__)
generator = Llama.build(
        ckpt_dir=LLAMA2['path'],
        tokenizer_path=LLAMA2['tokenizer'],
        max_seq_len=4096,
        max_batch_size=6,
    )

@app.post("/llama2")
def read_root():
    global generator
    print("Received request from task script")
    data = request.json
    dialogs = [data["messages"]]
    t1 = time.time()
    dist.broadcast_object_list([dialogs, 4096, 0])
    response = generator.chat_completion(
        dialogs,
        max_gen_len=4096,
        temperature=0
    )
    t2 = time.time()
    print(f"Code generation took {t2-t1} seconds")
    return response

if __name__ == '__main__':
    if dist.get_rank() == 0:
        app.run(debug=True, port=6969, use_reloader=False)