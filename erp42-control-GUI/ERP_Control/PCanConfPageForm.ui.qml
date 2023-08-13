import QtQuick 2.11
import QtQuick.Controls 2.4
import Qt.labs.calendar 1.0
import QtQuick.Dialogs.qml 1.0

Page {
    id: page
    width: 1600
    height: 760

    readonly property int steer_MIN_ANGLE: -28
    readonly property int steer_MAX_ANGLE: 28
    readonly property int speed_MIN: 0
    readonly property int speed_MAX: 20
    readonly property int brake_MIN: 0
    readonly property int brake_MAX: 100

    //Enable
    property alias activeCAN: switchActive.checked
    property alias autoControl: switchAuto.checked
    property alias estopControl: switchEstop.checked
    property alias steerControl: switchSteerControl.checked
    property alias speedControl: switchSpeedControl.checked
    property alias brakeControl: switchBrakeControl.checked

    // GEAR
    property alias gearDrive: rbuttonDrive.checked
    property alias gearNeutral: rbuttonNeutral.checked
    property alias gearReverse: rbuttonReverse.checked

    // Value
    property alias steerAngle: spinBoxSteerAngle.value
    property alias speed: spinBoxSpeed.value
    property alias brake: spinBoxBrake.value

    property alias cycle: spinBoxCycle.value

    //    property alias mode : switchAuto.checked
    header: Label {
        text: qsTr("ERP42 Control Panel")
        horizontalAlignment: Text.AlignHCenter
        font.pixelSize: Qt.application.font.pixelSize * 2
    }

    Switch {
        id: switchActive
        x: 28
        y: 6
        text: qsTr("Active")
    }

    Column {
        x: 28
        y: 46
        width: 550
        height: 100
        spacing: 10

        Row {
            width: 550
            height: 30
            spacing: 20

            Switch {
                id: switchAuto
                enabled: switchActive.checked
                checked: switchActive.checked
                text: qsTr("AUTO")
                anchors.verticalCenter: parent.verticalCenter
            }

            Switch {
                id: switchEstop
                enabled: switchActive.checked
                checked: switchActive.checked
                text: qsTr("ESTOP")
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        Row {
            width: 550
            height: 30
            spacing: 20

            Switch {
                id: switchSteerControl
                enabled: switchActive.checked
                checked: switchActive.checked
                text: qsTr("Steer Control")
                anchors.verticalCenter: parent.verticalCenter
            }

            Switch {
                id: switchSpeedControl
                enabled: switchActive.checked
                checked: switchActive.checked
                text: qsTr("Speed Control")
                anchors.verticalCenter: parent.verticalCenter
            }

            Switch {
                id: switchBrakeControl
                enabled: switchActive.checked
                checked: switchActive.checked
                text: qsTr("Brake Control")
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        Row {
            width: 550
            height: 30
            spacing: 20

            Text {
                id: textGear
                text: qsTr("GEAR")
                anchors.verticalCenter: parent.verticalCenter
            }

            RadioButton {
                id: rbuttonDrive
                enabled: switchActive.checked
                text: qsTr("D")
            }

            RadioButton {
                id: rbuttonNeutral
                enabled: switchActive.checked
                checked: switchActive.checked
                text: qsTr("N")
            }
            RadioButton {
                id: rbuttonReverse
                enabled: switchActive.checked
                text: qsTr("R")
            }
        }
    }

    Row {
        x: 28
        y: 200
        width: 605
        height: 280

        Column {
            id: columnSteerAngle
            width: 200
            height: 280
            spacing: 10

            Text {
                text: qsTr("Steer Angle[deg]")
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 12
            }

            SpinBox {
                id: spinBoxSteerAngle
                editable: true
                enabled: switchSteerControl.checked
                stepSize: 1
                from: steer_MIN_ANGLE
                to: steer_MAX_ANGLE
                value: dialSteerAngle.value
                anchors.horizontalCenter: parent.horizontalCenter
            }

            Dial {
                id: dialSteerAngle
                width: 200
                height: 200
                visible: true
                inputMode: Dial.Circular
                stepSize: 1
                wheelEnabled: true
                from: steer_MIN_ANGLE
                to: steer_MAX_ANGLE
                value: dialSteerAngle.value //steerAngle
                enabled: switchSteerControl.checked
                anchors.horizontalCenter: parent.horizontalCenter

                Text {
                    id: textsteerMin
                    text: "-28"
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    font.pixelSize: 12
                }

                Text {
                    id: textsteerMax
                    text: "28"
                    anchors.bottom: parent.bottom
                    anchors.right: parent.right
                    font.pixelSize: 12
                }
            }
        }

        Column {
            id: columnSpeed
            width: 200
            height: 280
            Text {
                id: textSpeed
                text: qsTr("Speed[KPH]")
                font.pixelSize: 12
                anchors.horizontalCenter: parent.horizontalCenter
            }

            SpinBox {
                id: spinBoxSpeed
                to: speed_MAX
                from: speed_MIN
                enabled: switchSpeedControl.checked
                editable: true
                stepSize: 1
                value: sliderSpeed.value
                anchors.horizontalCenter: parent.horizontalCenter
            }

            Slider {
                id: sliderSpeed
                to: speed_MAX
                from: speed_MIN
                enabled: switchSpeedControl.checked
                anchors.horizontalCenter: parent.horizontalCenter
                stepSize: 1
                orientation: Qt.Vertical
                value: sliderSpeed.value

                Text {
                    text: "0"
                    anchors.bottom: parent.bottom
                    anchors.left: parent.right
                    font.pixelSize: 12
                }

                Text {
                    text: "20"
                    anchors.top: parent.top
                    anchors.left: parent.right
                    font.pixelSize: 12
                }
            }

            spacing: 10
        }

        Column {
            id: columnAccel
            width: 200
            height: 280
            spacing: 10

            Text {
                text: qsTr("Brake")
                anchors.horizontalCenter: parent.horizontalCenter
                font.pixelSize: 12
            }

            SpinBox {
                id: spinBoxBrake
                to: brake_MAX
                from: brake_MIN
                value: sliderBrake.value
                enabled: switchBrakeControl.checked
                stepSize: 1
                anchors.horizontalCenter: parent.horizontalCenter
                editable: true
            }

            Slider {
                id: sliderBrake
                to: brake_MAX
                from: brake_MIN
                anchors.horizontalCenter: parent.horizontalCenter
                enabled: switchBrakeControl.checked
                stepSize: 1
                orientation: Qt.Vertical
                value: sliderBrake.value
                Text {
                    text: "0"
                    anchors.bottom: parent.bottom
                    anchors.left: parent.right
                    font.pixelSize: 12
                }

                Text {
                    text: "100"
                    anchors.top: parent.top
                    anchors.left: parent.right
                    font.pixelSize: 12
                }
            }
        }
    }

//    Text {
//        id: textRawData
//        x: 690
//        y: 25
//        text: qsTr("Console Output")
//        font.pixelSize: 12
//    }

//    TextArea {
//        id: textAreaRawData
//        x: 690
//        y: 55
//        width: 573
//        height: 44
//        text: qsTr(canManager.TextArea)
//        renderType: Text.NativeRendering
//        verticalAlignment: Text.AlignTop
//        textFormat: Text.AutoText
//        wrapMode: Text.WrapAnywhere
//        readOnly: true

//        //            TextEdit: canManager.TextArea
//    }

    Row {
        id: row5
        x: 28
        y: 517
        width: 800
        height: 100
        spacing: 20

        Column {
            width: 50
            height: 50
            spacing: 10

            Text {

                id: textMorA
                width: 50
                horizontalAlignment: Text.AlignHCenter
                text: qsTr("MorA")
            }
            TextArea {
                id: textAreaMorA
                width: 50
                height: 30
                enabled: switchActive.checked
                text: qsTr(pcanManager.get_QMorA)
                readOnly: true
            }
        }
        Column {
            width: 50
            height: 50
            spacing: 10
            Text {
                id: textestop
                width: 50
                horizontalAlignment: Text.AlignHCenter
                text: qsTr("E-STOP")
            }
            TextArea {
                id: textAreaestop
                width: 50
                height: 30
                enabled: switchActive.checked
                text: qsTr(pcanManager.get_ESTOP)
                readOnly: true
            }
        }
        Column {
            width: 50
            height: 50
            spacing: 10
            Text {
                id: textgear
                width: 50
                horizontalAlignment: Text.AlignHCenter
                text: qsTr("GEAR")
            }
            TextArea {
                id: textAreagear
                width: 50
                height: 30
                enabled: switchActive.checked
                text: qsTr(pcanManager.get_GEAR)
                readOnly: true
            }
        }
        Column {
            width: 100
            height: 50
            spacing: 10
            Text {
                id: textspeed
                width: 100
                horizontalAlignment: Text.AlignHCenter
                text: qsTr("Speed")
            }
            TextArea {
                id: textAreaspeed
                width: 100
                height: 30
                enabled: switchSpeedControl.checked
                text: qsTr(pcanManager.get_SPEED)
                readOnly: true
            }
            TextArea {
                id: textAreaspeed1
                width: 100
                height: 30
                enabled: switchSpeedControl.checked
                text: qsTr(pcanManager.get_modified_SPEED)
                readOnly: true
            }
        }
        Column {
            width: 100
            height: 50
            spacing: 10

            Text {
                id: textSteer
                width: 100
                horizontalAlignment: Text.AlignHCenter
                text: qsTr("Steer")
            }
            TextArea {
                id: textAreaSteer
                width: 100
                height: 30
                enabled: switchSteerControl.checked
                text: qsTr(pcanManager.get_STEER)
                readOnly: true
            }
            TextArea {
                id: textAreaSteer1
                width: 100
                height: 30
                text: qsTr(pcanManager.get_modified_STEER)
                enabled: switchSteerControl.checked
                readOnly: true
            }
        }
        Column {
            width: 100
            height: 50
            spacing: 10
            Text {
                id: textBrake
                width: 100
                horizontalAlignment: Text.AlignHCenter
                text: qsTr("Brake")
            }
            TextArea {
                id: textAreaBrake
                width: 100
                height: 30
                text: qsTr(pcanManager.get_BRAKE)
                enabled: switchBrakeControl.checked
                readOnly: true
            }
            TextArea {
                id: textAreaBrake1
                width: 100
                height: 30
                text: qsTr(pcanManager.get_modified_BRAKE)
                enabled: switchBrakeControl.checked
                readOnly: true
            }
        }
        Column {
            width: 50
            height: 50
            spacing: 10
            Text {
                id: textAlive
                width: 50
                horizontalAlignment: Text.AlignHCenter
                text: qsTr("Alive")
            }
            TextArea {
                id: textAreaAlive
                width: 50
                height: 30
                enabled: switchActive.checked
                text: qsTr(pcanManager.get_ALIVE)
                readOnly: true
            }
        }
    }

    SpinBox {
        id: spinBoxCycle
        x: 447
        y: 25
        width: 117
        height: 33
        to: 5000
        value: 20
        editable: true
        stepSize: 10
    }

    Text {
        id: textCycle
        x: 344
        y: 31
        width: 90
        height: 27
        text: "Loop Cycle [ms]"
        font.pixelSize: 12
    }

    Rectangle {
        id: frameFeedback
        x: 690
        y: 200
        width: 853
        height: 280
        border.color: "black"

        Row {
            id: rowFB1
            x: 102
            y: 62
            width: 627
            height: 102
            spacing: 20

            Column {
                width: 50
                height: 50
                spacing: 10

                Text {

                    id: textFBMorA1
                    width: 50
                    horizontalAlignment: Text.AlignHCenter
                    text: qsTr("MorA")
                }
                TextArea {
                    id: textAreaFBMorA1
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_QMorA)
                    readOnly: true
                }
            }
            Column {
                width: 50
                height: 50
                spacing: 10
                Text {
                    id: texteFBstop1
                    width: 50
                    horizontalAlignment: Text.AlignHCenter
                    text: qsTr("E-STOP")
                }
                TextArea {
                    id: textAreaFBestop1
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_ESTOP)
                    readOnly: true
                }
            }
            Column {
                width: 50
                height: 50
                spacing: 10
                Text {
                    id: textFBgear1
                    width: 50
                    horizontalAlignment: Text.AlignHCenter
                    text: qsTr("GEAR")
                }
                TextArea {
                    id: textAreaFBgear1
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_GEAR)
                    readOnly: true
                }
            }
            Column {
                width: 100
                height: 50
                spacing: 10
                Text {
                    id: textFBspeed1
                    width: 100
                    horizontalAlignment: Text.AlignHCenter
                    text: qsTr("Speed")
                }
                TextArea {
                    id: textAreaFBspeed1
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_SPEED)
                    readOnly: true
                }
                TextArea {
                    id: textAreaFBspeed11
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_modified_SPEED)
                    readOnly: true
                }
            }
            Column {
                width: 100
                height: 50
                spacing: 10

                Text {
                    id: textFBSteer1
                    width: 100
                    horizontalAlignment: Text.AlignHCenter
                    text: qsTr("Steer")
                }
                TextArea {
                    id: textAreaFBSteer1
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_STEER)
                    readOnly: true
                }
                TextArea {
                    id: textAreaFBSteer11
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_modified_STEER)
                    readOnly: true
                }
            }
            Column {
                width: 100
                height: 50
                spacing: 10
                Text {
                    id: textFBBrake1
                    width: 100
                    horizontalAlignment: Text.AlignHCenter
                    text: qsTr("Brake")
                }
                TextArea {
                    id: textAreaFBBrake1
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_BRAKE)
                    readOnly: true
                }
                TextArea {
                    id: textAreaFBBrake11
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_modified_BRAKE)
                    readOnly: true
                }
            }
            Column {
                width: 50
                height: 50
                spacing: 10
                Text {
                    id: textFBAlive1
                    width: 50
                    horizontalAlignment: Text.AlignHCenter
                    text: qsTr("Alive")
                }
                TextArea {
                    id: textAreaFBAlive1
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_ALIVE)
                    readOnly: true
                }
            }
        }

        Text {
            id: element
            x: 384
            y: 14
            width: 86
            height: 26
            text: qsTr("Feedback")
            font.italic: true
            font.pixelSize: 21
        }

        Row {
            id: rowFeedback2
            x: 102
            y: 170
            width: 743
            height: 102
            Column {
                width: 100
                height: 50
                Text {
                    id: textFBEncoder0
                    width: 100
                    text: qsTr("Encoder0")
                    horizontalAlignment: Text.AlignHCenter
                }
                TextArea {
                    id: textAreaFBEncoder0
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_Encoder0)
                }
                TextArea {
                    id: textAreaFBEncoder00
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_modified_Encoder0)
                }

                spacing: 10
            }

            Column {
                width: 100
                height: 50
                Text {
                    id: textFBEncoder1
                    width: 100
                    text: qsTr("Encoder1")
                    horizontalAlignment: Text.AlignHCenter
                }

                TextArea {
                    id: textAreaFBEncoder1
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_Encoder1)
                    readOnly: true
                }
                TextArea {
                    id: textAreaFBEncoder11
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_modified_Encoder1)
                    readOnly: true
                }
                spacing: 10
            }

            Column {
                width: 100
                height: 50
                Text {
                    id: textFBEncoder2
                    width: 100
                    text: qsTr("Encoder2")
                    horizontalAlignment: Text.AlignHCenter
                }

                TextArea {
                    id: textAreaFBEncoder2
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_Encoder2)
                    readOnly: true
                }
                TextArea {
                    id: textAreaFBEncoder22
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_modified_Encoder2)
                    readOnly: true
                }
                spacing: 10
            }

            Column {
                width: 100
                height: 50
                Text {
                    id: textFBEncoder3
                    width: 100
                    text: qsTr("Encoder3")
                    horizontalAlignment: Text.AlignHCenter
                }

                TextArea {
                    id: textAreaFBEncoder3
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_Encoder3)
                    readOnly: true
                }
                TextArea {
                    id: textAreaFBEncoder33
                    width: 100
                    height: 30
                    text: qsTr(pcanManager.set_modified_Encoder3)
                    readOnly: true
                }
                spacing: 10
            }

            Column {
                width: 50
                height: 50
                Text {
                    id: textFBBRAKE_CMD_RAW
                    width: 50
                    text: qsTr("B.C.R")
                    horizontalAlignment: Text.AlignHCenter
                }

                TextArea {
                    id: textAreaFBBRAKE_CMD_RAW1
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_BCR)
                    readOnly: true
                }

                TextArea {
                    id: textAreaFBBRAKE_CMD_RAW11
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_modified_BCR)
                    readOnly: true
                }
                spacing: 10
            }

            Column {
                width: 50
                height: 50
                Text {
                    id: textFBBrake_RAW
                    width: 50
                    text: qsTr("B.R")
                    horizontalAlignment: Text.AlignHCenter
                }

                TextArea {
                    id: textAreaBrake_RAW1
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_BR)
                    readOnly: true
                }

                TextArea {
                    id: textAreaBrake_RAW11
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_modified_BR)
                    readOnly: true
                }
                spacing: 10
            }

            Column {
                width: 50
                height: 50
                Text {
                    id: textFBBrake_Echo
                    width: 50
                    text: qsTr("B.E")
                    horizontalAlignment: Text.AlignHCenter
                }

                TextArea {
                    id: textAreaBrake_Echo
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_BE)
                    readOnly: true
                }

                TextArea {
                    id: textAreaBrake_Echo1
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_modified_BE)
                    readOnly: true
                }
                spacing: 10
            }

            Column {
                width: 50
                height: 50
                Text {
                    id: textBrake_init_max
                    width: 50
                    text: qsTr("B.I.M")
                    horizontalAlignment: Text.AlignHCenter
                }

                TextArea {
                    id: textAreaBrake_init_max
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_BIM)
                    readOnly: true
                }

                TextArea {
                    id: textAreaBrake_init_max1
                    width: 50
                    height: 30
                    text: qsTr(pcanManager.set_modified_BIM)
                    readOnly: true
                }
                spacing: 10
            }
            spacing: 20
        }

        Column {
            id: columnFeedBackID
            x: 8
            y: 62
            width: 79
            height: 102

            Text {
                id: textFBID
                width: 79
                height: 17

                text: qsTr("ID")
                font.pointSize: 11
                horizontalAlignment: Text.AlignHCenter
            }

            TextArea {
                id: textAreaFBID
                y: 27
                width: 79
                height: 30
                text: qsTr(pcanManager.set_ID1)
            }
            spacing: 10
        }

        Column {
            id: columnFeedBackID2
            x: 8
            y: 170
            width: 79
            height: 102
            Text {
                id: etextFBID2
                width: 79
                height: 17
                text: qsTr("ID")
                font.pointSize: 11
                horizontalAlignment: Text.AlignHCenter
            }

            TextArea {
                id: textAreaFBID2
                y: 27
                width: 79
                height: 30
                text: qsTr(pcanManager.set_ID2)
            }
            spacing: 10
        }
    }

    Image {
        id: name
        x: 1215
        y: 486
        width: 171
        height: 177
        source: "../images/erp42.png"
    }
}




/*##^## Designer {
    D{i:58;anchors_height:33;anchors_width:90;anchors_x:439;anchors_y:25}D{i:90;anchors_x:306;anchors_y:8}
}
 ##^##*/
