import base64

import requests

cred = base64.b64encode(b"suave101:bb82dc12-2cba-484c-8e7e-8fc395b1326a")

data = requests.get(
    url="https://frc-api.firstinspires.org/v3.0/2023/teams",
    headers={"Authorization": f"Basic {cred}="}
)

print(cred)
print(data.status_code)
print(data.reason)
print(data.headers)
print(data.content)
