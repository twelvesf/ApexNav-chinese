import cv2
import numpy as np
from vlm.itm.blip2itm import BLIP2ITMClient

itmclient = BLIP2ITMClient(port=12182)

def get_itm_message(rgb_image, label):
    txt = f"Is there a {label} in the image?"
    cosine = itmclient.cosine(rgb_image, txt)
    itm_score = itmclient.itm_score(rgb_image, txt)
    return cosine, itm_score
# 把当前目标标签和房间上下文拼成一句自然语言提示词，
# 然后调用 BLIP2-ITM 计算这张图和这句话的语义相似度。
def get_itm_message_cosine(rgb_image, label, room):
    if room != "everywhere":
        txt = f"Seems like there is a {room} or a {label} ahead?"
    else:
        txt = f"Seems like there is a {label} ahead?"
    cosine = itmclient.cosine(rgb_image, txt)
    return cosine