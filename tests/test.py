# 单独使用各模块，返回值一般为字典，参数参考命令行
from pywxdump import *

# 获取微信信息
wx_info = read_info(VERSION_LIST, True)

# 获取微信文件夹路径
args = {
    "mode": "db_path",
    "require_list": "all",  # 需要的数据库名称（可选）
    "wx_files": "WeChat Files",  # 'WeChat Files'路径（可选）
    "wxid": "wxid_",  # wxid_，用于确认用户文件夹（可选）
}
user_dirs = get_wechat_db(args["require_list"], args["wx_files"], args["wxid"], True)
# ************************************************************************************************ #
# 解密微信数据库
args = {
    "mode": "decrypt",
    "key": "密钥",  # 密钥
    "db_path": "数据库路径(目录or文件)",  # 数据库路径
    "out_path": "/path/to/decrypted"  # 输出路径（必须是目录）[默认为当前路径下decrypted文件夹]
}
result = batch_decrypt(args["key"], args["db_path"], args["out_path"], True)
# ************************************************************************************************ #
# 查看聊天记录
args = {
    "mode": "dbshow",
    "msg_path": "解密后的 MSG.db 的路径",  # 解密后的 MSG.db 的路径
    "micro_path": "解密后的 MicroMsg.db 的路径",  # 解密后的 MicroMsg.db 的路径
    "media_path": "解密后的 MediaMSG.db 的路径",  # 解密后的 MediaMSG.db 的路径
    "filestorage_path": "文件夹FileStorage的路径"  # 文件夹 FileStorage 的路径（用于显示图片）
}
from flask import Flask, request, jsonify, render_template, g
import logging

app = Flask(__name__, template_folder='./show_chat/templates')
app.logger.setLevel(logging.ERROR)


@app.before_request
def before_request():
    g.MSG_ALL_db_path = args["msg_path"]
    g.MicroMsg_db_path = args["micro_path"]
    g.MediaMSG_all_db_path = args["media_path"]
    g.FileStorage_path = args["filestorage_path"]
    g.USER_LIST = get_user_list(args["msg_path"], args["micro_path"])


app.register_blueprint(app_show_chat)
print("[+] 请使用浏览器访问 http://127.0.0.1:5000/ 查看聊天记录")
app.run(debug=False)
# ************************************************************************************************ #
# 导出聊天记录为 HTML
args = {
    "mode": "export",
    "username": "微信账号",  # 微信账号（聊天对象账号）
    "outpath": "/path/to/export",  # 导出路径
    "msg_path": "解密后的 MSG.db 的路径",  # 解密后的 MSG.db 的路径
    "micro_path": "解密后的 MicroMsg.db 的路径",  # 解密后的 MicroMsg.db 的路径
    "media_path": "解密后的 MediaMSG.db 的路径",  # 解密后的 MediaMSG.db 的路径
    "filestorage_path": "文件夹FileStorage的路径"  # 文件夹 FileStorage 的路径（用于显示图片）
}
export(args["username"], args["outpath"], args["msg_path"], args["micro_path"], args["media_path"],
       args["filestorage_path"])