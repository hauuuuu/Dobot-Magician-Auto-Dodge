import DobotDllType as dType

# 目的地
dest_x = 160.0
dest_y = -140.0
dest_z = 20.0
dest_r = 0.0

# 是否偵測到障礙物
detect_flag = 0

# Dobot 連接狀態
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

# 載入dll
api = dType.load()

# 連接Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:", CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):

    print("Dobot Connected")

    # 清除當前queue中的指令
    dType.SetQueuedCmdClear(api)
    print("Queue cleared")

    # 設置運動參數
    dType.SetPTPJointParams(api, 50, 50, 50, 50, 50, 50, 50, 50, isQueued=1)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    dType.SetPTPJumpParams(api, 40, 150, isQueued=1)

    # 移動到起始點位置
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode,
                    160, 140, 20, 0, isQueued=1)
    dType.SetWAITCmd(api, 1000, isQueued=1)

    # 移動到目的地位置
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode,
                    dest_x, dest_y, dest_z, dest_r, isQueued=1)
    dType.SetWAITCmd(api, 1000, isQueued=1)

    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)

    # 開始執行Queue中的指令
    dType.SetQueuedCmdStartExec(api)
    print("Queue start")

    # 檢查是否執行到最後的指令
    while lastIndex[0] > dType.GetQueuedCmdCurrentIndex(api)[0]:

        # 偵測是否有障礙物
        detect = dType.GetIODI(api, 5)

        # 如果偵測到障礙物則停止移動
        if detect[0] == 0:
            dType.SetQueuedCmdForceStopExec(api)
            detect_flag = 1
            print("detected!")
            break

        # 為了確保指令完整執行
        dType.dSleep(100)

    # 停止執行指令
    dType.SetQueuedCmdStopExec(api)
    print("Queue stop")

    dType.SetQueuedCmdClear(api)
    print("Queue cleared")

    # 執行閃避策略
    while detect_flag == 1:

        detect_flag = 0

        cur_pose = dType.GetPose(api)
        cur_x = cur_pose[0]
        cur_y = cur_pose[1]
        cur_z = cur_pose[2]
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode,
                        cur_x, cur_y, cur_z+30.0, 0, isQueued=1)
        lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)

        dType.SetQueuedCmdStartExec(api)

        while lastIndex[0] > dType.GetQueuedCmdCurrentIndex(api)[0]:
            # 為了確保指令完整執行
            dType.dSleep(100)

        print("move up done")
        dType.SetQueuedCmdStopExec(api)
        print("Queue stop")

        detect = dType.GetIODI(api, 5)
        if detect[0] == 0:
            detect_flag = 1
            print("detected!")
            dType.dSleep(100)
            continue
        else:
            print("no detected")
            dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode,
                            dest_x, dest_y, dest_z, dest_r, isQueued=1)
            lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)

            dType.SetQueuedCmdStartExec(api)

            while lastIndex[0] > dType.GetQueuedCmdCurrentIndex(api)[0]:
                # 為了確保指令完整執行
                print(dType.GetQueuedCmdCurrentIndex(api)[0])
                dType.dSleep(100)

# 停止執行指令
dType.SetQueuedCmdStopExec(api)
print("Queue stop")

dType.SetQueuedCmdClear(api)
print("Queue cleared")

# 斷開連接
dType.DisconnectDobot(api)
print("Dobot disconnected")
