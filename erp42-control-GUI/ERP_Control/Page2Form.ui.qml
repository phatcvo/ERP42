import QtQuick 2.12
import QtQuick.Controls 2.5
import QtQuick.Extras 1.4

Page {
    width: 600
    height: 400

    header: Label {
        text: qsTr("Page 2")
        font.pixelSize: Qt.application.font.pixelSize * 2
        padding: 10
    }

    CircularGauge {
        id: circularGauge
        x: 189
        y: 48
        width: 223
        height: 202
    }
}
