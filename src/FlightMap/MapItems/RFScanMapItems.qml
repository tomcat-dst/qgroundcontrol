import QtQuick
import QtLocation
import QtPositioning

import QGroundControl
import QGroundControl.FlightMap

/// RF scan demo (cert-env): the nose-mounted antenna's beam as a cone on the map, coloured by the
/// companion's received power (Vehicle.rfScanSample, NAMED_VALUE_FLOAT "RF_dBm"): green = nothing or
/// below the detection threshold validDbm, then yellow to red as the valid reading gets stronger.
/// Each cone is drawn in bands that fade with distance from the aircraft. The live cone follows the
/// aircraft; cones left behind along the track fade out over trailSeconds, and a small dot per sample
/// position stays as the power record of the flight. The companion's fitted bearings (Vehicle.rfBearing,
/// DEBUG_FLOAT_ARRAY "RF_BRG": each peak above the threshold in one observation sweep) are drawn as
/// vectors from where they were taken, and its source fixes (Vehicle.rfFix, "RF_FIX") as their 1-sigma
/// error ellipses. Each source has its own colour, and so do the bearings assigned to it
/// (grey: not pointing at any source); fixes appear and move as the companion updates them.
Item {
    id: _root

    property var    map                                                         ///< Map control to show items on
    property var    vehicle:        QGroundControl.multiVehicleManager.activeVehicle
    property real   beamDeg:        30      ///< full -3 dB beam width (nose helix)
    property real   coneLengthM:    2500    ///< drawn length; the beam itself has no range limit
    property int    bands:          5       ///< cone segments, fading outward
    property real   validDbm:       -95     ///< detection threshold (15 dB above the analyzer floor)
    property real   floorDbm:       validDbm ///< drawn green (and everything below)
    property real   strongDbm:      -68     ///< drawn red
    property real   minStepM:       60      ///< new trail cone / dot after this much travel ...
    property real   minTurnDeg:     8       ///< ... or this much turn
    property real   trailSeconds:   5
    property real   dotRadiusM:     30
    property real   vectorLengthM:  15000
    property real   labelOffsetM:   1500    ///< bearing captions sit this far out along the vector, clear of the waypoint at its base

    property var    _lastCoord:     QtPositioning.coordinate()
    property real   _lastHeading:   NaN
    property real   _liveDbm:       NaN
    property var    _liveCoord:     vehicle ? vehicle.coordinate : QtPositioning.coordinate()
    property real   _liveHeading:   vehicle ? vehicle.heading.rawValue : NaN
    property bool   _liveValid:     !isNaN(_liveDbm) && _liveCoord.isValid && !isNaN(_liveHeading)
    property real   _now:           Date.now()
    property int    _fixRev:        0       ///< bumped on every fix change, so truth labels re-measure

    function powerFraction(dbm) {
        return Math.max(0, Math.min(1, (dbm - floorDbm) / (strongDbm - floorDbm)))
    }

    function powerColor(dbm, alpha) {
        return Qt.hsla((1 - powerFraction(dbm)) / 3, 1, 0.5, alpha)
    }

    /// band 0 is the apex triangle, band n an annular sector between n and n+1 band lengths
    function bandPath(coord, heading, band) {
        const r0 = coneLengthM * band / bands
        const r1 = coneLengthM * (band + 1) / bands
        const steps = 6
        const path = []
        for (let i = 0; i <= steps; i++) {
            path.push(coord.atDistanceAndAzimuth(r1, heading - beamDeg / 2 + beamDeg * i / steps))
        }
        if (band === 0) {
            path.push(coord)
        } else {
            for (let i = steps; i >= 0; i--) {
                path.push(coord.atDistanceAndAzimuth(r0, heading - beamDeg / 2 + beamDeg * i / steps))
            }
        }
        return path
    }

    function bandAlpha(band) {
        return 1 - band / bands
    }

    function _headingDelta(a, b) {
        return Math.abs(((a - b + 540) % 360) - 180)
    }

    function _clear() {
        _trail.clear()
        _dots.clear()
        _vectors.clear()
        _fixes.clear()
        _truth.clear()
        _fixRev++
        _lastCoord = QtPositioning.coordinate()
        _liveDbm = NaN
    }

    function _addSample(dbm) {
        if (!vehicle) {
            return
        }
        const coord = vehicle.coordinate
        const heading = vehicle.heading.rawValue
        if (!coord.isValid || isNaN(heading)) {
            return
        }
        _liveDbm = dbm
        if (_dots.count > 0 && _lastCoord.isValid && coord.distanceTo(_lastCoord) < minStepM
                && _headingDelta(heading, _lastHeading) < minTurnDeg) {
            const last = _dots.count - 1
            if (dbm > _dots.get(last).dbm) {
                _dots.setProperty(last, "dbm", dbm)
            }
            return
        }
        const t = Date.now()
        for (let b = 0; b < bands; b++) {
            _trail.append({ "lat": coord.latitude, "lon": coord.longitude, "heading": heading, "dbm": dbm,
                            "band": b, "t": t })
        }
        _dots.append({ "lat": coord.latitude, "lon": coord.longitude, "dbm": dbm })
        _lastCoord = coord
        _lastHeading = heading
    }

    /// colour of source n (1-based); 0 (bearing not pointing at any source) is grey. Avoids the
    /// green-to-red power scale.
    function sourceColor(n, alpha) {
        const palette = [[0.45, 0.80, 1.0], [1.0, 0.74, 0.30], [0.84, 0.60, 1.0], [0.35, 0.95, 0.86],
                         [1.0, 0.55, 0.84], [0.88, 0.93, 0.35]]
        if (n <= 0) {
            return Qt.rgba(0.92, 0.92, 0.92, alpha)
        }
        const c = palette[(n - 1) % palette.length]
        return Qt.rgba(c[0], c[1], c[2], alpha)
    }

    function _addBearing(id, lat, lon, bearing, dbm, sigma, observation, source, wide) {
        const v = { "id": id, "lat": lat, "lon": lon, "heading": bearing, "dbm": dbm, "sigma": sigma, "n": observation,
                    "source": source, "wide": wide ? 1 : 0 }
        let k = 1
        for (let i = 0; i < _vectors.count; i++) {
            const e = _vectors.get(i)
            if (e.id === id) {
                _vectors.setProperty(i, "source", source)   // re-sent: only the assignment can change
                return
            }
            if (e.n === observation) {
                k++
            }
        }
        v.k = k
        _vectors.append(v)
    }

    function _setFix(source, lat, lon, majorM, minorM, orientDeg, bearings) {
        _fixRev++
        const fix = { "source": source, "lat": lat, "lon": lon, "major": majorM, "minor": minorM, "orient": orientDeg,
                      "bearings": bearings }
        for (let i = 0; i < _fixes.count; i++) {
            if (_fixes.get(i).source === source) {
                if (majorM < 0) {
                    _fixes.remove(i)
                } else {
                    _fixes.set(i, fix)
                }
                return
            }
        }
        if (majorM >= 0) {
            _fixes.append(fix)
        }
    }

    /// simulation only: true source positions (companion RF_TRU), kept until the next arm
    function _setTruth(index, lat, lon) {
        for (let i = 0; i < _truth.count; i++) {
            if (_truth.get(i).index_ === index) {
                _truth.set(i, { "index_": index, "lat": lat, "lon": lon })
                return
            }
        }
        _truth.append({ "index_": index, "lat": lat, "lon": lon })
    }

    /// "<distance> m to S<n>" for the fix nearest to coord, or "" with no fix yet
    function nearestFixText(coord, rev) {
        let best = -1, bestSource = 0
        for (let i = 0; i < _fixes.count; i++) {
            const f = _fixes.get(i)
            const d = coord.distanceTo(QtPositioning.coordinate(f.lat, f.lon))
            if (best < 0 || d < best) {
                best = d
                bestSource = f.source
            }
        }
        return best < 0 ? "" : " \u00b7 " + Math.round(best) + " m to S" + bestSource
    }

    /// 1-sigma error ellipse around centre: semi-axes in m (each at least 50 m so it stays visible),
    /// major axis along azimuth orientDeg
    function ellipsePath(centre, majorM, minorM, orientDeg) {
        const a = Math.max(majorM, 50)
        const b = Math.max(minorM, 50)
        const path = []
        for (let i = 0; i < 48; i++) {
            const phi = 2 * Math.PI * i / 48
            const u = a * Math.cos(phi)
            const v = b * Math.sin(phi)
            path.push(centre.atDistanceAndAzimuth(Math.hypot(u, v), orientDeg + Math.atan2(v, u) * 180 / Math.PI))
        }
        return path
    }

    ListModel { id: _trail }
    ListModel { id: _vectors }
    ListModel { id: _fixes }
    ListModel { id: _truth }    // role "index_": "index" collides with the delegate's index
    ListModel { id: _dots }
    ListModel {
        id: _liveBands
        Component.onCompleted: {
            for (let b = 0; b < _root.bands; b++) {
                append({ "band": b })
            }
        }
    }

    Timer {
        interval:   250
        running:    _trail.count > 0
        repeat:     true
        onTriggered: {
            _root._now = Date.now()
            while (_trail.count > 0 && _root._now - _trail.get(0).t > _root.trailSeconds * 1000) {
                _trail.remove(0)
            }
        }
    }

    Connections {
        target:                 _root.vehicle
        ignoreUnknownSignals:   true

        function onRfScanSample(dbm) { _root._addSample(dbm) }
        function onRfBearing(id, lat, lon, bearing, dbm, sigma, observation, source, wide) {
            _root._addBearing(id, lat, lon, bearing, dbm, sigma, observation, source, wide)
        }
        function onRfFix(source, lat, lon, majorM, minorM, orientDeg, bearings) {
            _root._setFix(source, lat, lon, majorM, minorM, orientDeg, bearings)
        }
        function onRfTruth(index, lat, lon) { _root._setTruth(index, lat, lon) }

        function onArmedChanged(armed) {
            if (armed) {
                _root._clear()
            }
        }
    }

    MapItemView {
        id:     _dotView
        model:  _dots

        delegate: MapCircle {
            center:         QtPositioning.coordinate(model.lat, model.lon)
            radius:         _root.dotRadiusM
            color:          _root.powerColor(model.dbm, 0.9)
            border.width:   0
            z:              9
        }
    }

    MapItemView {
        id:     _trailView
        model:  _trail

        delegate: MapPolygon {
            path:           _root.bandPath(QtPositioning.coordinate(model.lat, model.lon), model.heading, model.band)
            color:          _root.powerColor(model.dbm, 0.3 * _root.bandAlpha(model.band)
                                             * Math.max(0, 1 - (_root._now - model.t) / (_root.trailSeconds * 1000)))
            border.width:   0
            z:              10 + _root.powerFraction(model.dbm)
        }
    }

    MapItemView {
        id:     _liveView
        model:  _liveBands

        delegate: MapPolygon {
            visible:        _root._liveValid
            path:           _root._liveValid ? _root.bandPath(_root._liveCoord, _root._liveHeading, model.band) : []
            color:          _root.powerColor(_root._liveDbm, 0.6 * _root.bandAlpha(model.band))
            border.width:   0
            z:              12
        }
    }

    // each bearing's +-1 sigma as a faint wedge: a weak, smeared peak shows as a wide fan, a sharp one
    // as a sliver, so a wide bearing passing a fix at a distance reads as what it is
    MapItemView {
        id:     _vectorWedgeView
        model:  _vectors

        delegate: MapPolygon {
            readonly property var _from: QtPositioning.coordinate(model.lat, model.lon)
            path: {
                const p = [_from]
                for (let i = 0; i <= 8; i++) {
                    p.push(_from.atDistanceAndAzimuth(_root.vectorLengthM, model.heading - model.sigma + 2 * model.sigma * i / 8))
                }
                return p
            }
            color:          _root.sourceColor(model.source, 0.12)
            border.width:   0
            z:              12
        }
    }

    MapItemView {
        id:     _vectorLineView
        model:  _vectors

        delegate: MapPolyline {
            readonly property var _from: QtPositioning.coordinate(model.lat, model.lon)
            path:           [_from, _from.atDistanceAndAzimuth(_root.vectorLengthM, model.heading)]
            line.width:     model.source > 0 ? 2 : 1.5
            line.color:     _root.sourceColor(model.source, model.source > 0 ? 1 : 0.8)
            z:              model.source > 0 ? 14 : 13
        }
    }

    MapItemView {
        id:     _vectorArrowView
        model:  _vectors

        delegate: MapLineArrow {
            readonly property var _from: QtPositioning.coordinate(model.lat, model.lon)
            fromCoord:      _from
            toCoord:        _from.atDistanceAndAzimuth(_root.vectorLengthM, model.heading)
            arrowColor:     _root.sourceColor(model.source, 1)
            arrowPosition:  2
            z:              14
        }
    }

    MapItemView {
        id:     _vectorLabelView
        model:  _vectors

        delegate: MapQuickItem {
            coordinate:     QtPositioning.coordinate(model.lat, model.lon).atDistanceAndAzimuth(_root.labelOffsetM, model.heading)
            anchorPoint.x:  0
            anchorPoint.y:  _label.height
            z:              15
            sourceItem: Rectangle {
                width:  _label.width + 8
                height: _label.height + 4
                radius: 3
                color:  Qt.rgba(0, 0, 0, 0.65)
                Text {
                    id:     _label
                    x:      4
                    y:      2
                    color:  _root.sourceColor(model.source, 1)
                    text:   (model.source > 0 ? "S" + model.source + " " : "") + "B" + model.n + "." + model.k + "  " + Math.round(model.heading) + "\u00b0  " + model.dbm.toFixed(0) + " dBm" +
                            (model.wide ? "  \u2022 several sources?" : "")
                    font.bold: true
                }
            }
        }
    }

    MapItemView {
        id:     _fixCircleView
        model:  _fixes

        delegate: MapPolygon {
            path:           _root.ellipsePath(QtPositioning.coordinate(model.lat, model.lon), model.major, model.minor, model.orient)
            color:          _root.sourceColor(model.source, 0.25)
            border.color:   _root.sourceColor(model.source, 1)
            border.width:   3
            z:              16
        }
    }

    MapItemView {
        id:     _fixLabelView
        model:  _fixes

        delegate: MapQuickItem {
            coordinate:     QtPositioning.coordinate(model.lat, model.lon)
            anchorPoint.x:  -10
            anchorPoint.y:  _fixLabel.height / 2
            z:              17
            sourceItem: Rectangle {
                width:  _fixLabel.width + 8
                height: _fixLabel.height + 4
                radius: 3
                color:  Qt.rgba(0, 0, 0, 0.7)
                Text {
                    id:     _fixLabel
                    x:      4
                    y:      2
                    color:  _root.sourceColor(model.source, 1)
                    text:   "S" + model.source + "  " + Math.round(model.major) + "\u00d7" + Math.round(model.minor) + " m \u00b7 " + model.bearings + " brg"
                    font.bold: true
                }
            }
        }
    }

    // true source positions: white crosshair in a ring, centred exactly on the position, above the fixes
    MapItemView {
        id:     _truthView
        model:  _truth

        delegate: MapQuickItem {
            coordinate:     QtPositioning.coordinate(model.lat, model.lon)
            anchorPoint.x:  _truthMark.width / 2
            anchorPoint.y:  _truthMark.height / 2
            z:              18
            sourceItem: Item {
                id:     _truthMark
                width:  28
                height: 28
                Rectangle {
                    anchors.fill:   parent
                    radius:         width / 2
                    color:          "transparent"
                    border.color:   "white"
                    border.width:   2
                }
                Rectangle { x: parent.width / 2 - 1; y: -6; width: 2; height: parent.height + 12; color: "white" }
                Rectangle { x: -6; y: parent.height / 2 - 1; width: parent.width + 12; height: 2; color: "white" }
                Rectangle { x: parent.width / 2 - 2; y: parent.height / 2 - 2; width: 4; height: 4; color: "black" }
                Rectangle {
                    x:      parent.width + 8
                    y:      -_truthLabel.height - 2
                    width:  _truthLabel.width + 8
                    height: _truthLabel.height + 4
                    radius: 3
                    color:  Qt.rgba(1, 1, 1, 0.85)
                    Text {
                        id:     _truthLabel
                        x:      4
                        y:      2
                        color:  "black"
                        font.bold: true
                        text:   "T" + model.index_ + " true" +
                                _root.nearestFixText(QtPositioning.coordinate(model.lat, model.lon), _root._fixRev)
                    }
                }
            }
        }
    }

    Component.onCompleted: {
        if (map) {
            map.addMapItemView(_dotView)
            map.addMapItemView(_trailView)
            map.addMapItemView(_liveView)
            map.addMapItemView(_vectorWedgeView)
            map.addMapItemView(_vectorLineView)
            map.addMapItemView(_vectorArrowView)
            map.addMapItemView(_vectorLabelView)
            map.addMapItemView(_fixCircleView)
            map.addMapItemView(_fixLabelView)
            map.addMapItemView(_truthView)
        }
    }
}
