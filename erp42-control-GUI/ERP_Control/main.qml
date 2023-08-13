import QtQuick 2.12
import QtQuick.Controls 2.5
import unmansol.erp42.pcanmanager 0.1
import unmansol.erp42.canmanager 0.1

ApplicationWindow {
    id: root
    visible: true
    width: 1600
    height: 760
    title: qsTr("UNMANNED ERP42 CONTROL")


    PCanManager {
        id: pcan
    }

    CanManager{
        id: canmanager
        frameID: textInputID.text
        frameData: textInputDATA.text
    }

    SwipeView {
        id: swipeView
        anchors.fill: parent
        currentIndex: tabBar.currentIndex

        PCanConfPage {
        }

//        Page2Form {
//        }
    }

    //    ComboBox {
    //        id: comboBox1
    //        x: 660
    //        y: 54
    //        model: comboModel.comboList
    //        onActivated: {
    //            console.log("combomodel activated" + comboBox1.currentIndex)
    //            comboModel.currentIndex = comboBox1.currentIndex
    //         }

    //    }


    Text {
        id: textID
        x: 710
        y: 200
        width: 100
        height: 40
        font.pixelSize: 12
        text: "Frame ID"


    }

//    x: 22
//    y: 200
//    width: 605
//    height: 280

//    Column {
//        id: columnSteerAngle
//        width: 200
//        height: 280
//        spacing: 10

    Image {
        id: name
        x: 0
        y: 300
        width: 250
        height: 250
        source: "../images/handle.png"
        antialiasing: true
        anchors.centerIn: parent.Center
//        //                    source: "needle.png"
        transform: Rotation { origin.x: 125; origin.y: 125; angle: pcanManager.SteerAngle}
    }

    TextField {
        id: textInputID
        x: 690
        y: 180
        width: 100
        height: 40
        font.pixelSize: 12
        placeholderText: "Frame ID"
        validator: RegExpValidator { regExp: /[0-9a-fA-F]{3}/}
    }

    Text {
        id: textData
        x: 930
        y: 160
        width: 100
        height: 40
        font.pixelSize: 12
        text: "DATA"

    }

    TextField {
        id: textInputDATA
        x: 800
        y: 180
        width: 300
        height: 40
        font.pixelSize: 12
        placeholderText: "Data"
        validator: RegExpValidator { regExp: /[0-9a-fA-F]{2}(?: [0-9a-fA-F]{2})+$/}

    }

    Button {
        id: proccessButton
        x: 1120
        y: 180
        width: 70
        height: 40
        text: "Send"
        onClicked: canmanager.buttontest()
    }


    footer: TabBar {
        id: tabBar
        currentIndex: swipeView.currentIndex

        TabButton {
            text: qsTr("PCAN Conf")
        }
        TabButton {
            text: qsTr("Page 2")
        }
    }
}
