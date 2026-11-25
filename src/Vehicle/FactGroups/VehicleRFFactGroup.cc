#include "VehicleRFFactGroup.h"
#include "Vehicle.h"

VehicleRFFactGroup::VehicleRFFactGroup(QObject *parent)
    : FactGroup(250, QStringLiteral(":/json/Vehicle/RFFact.json"), parent)
{
    _addFact(&_dbmFact);
    _addFact(&_sweepFact);
    _addFact(&_sweepsFact);
    _addFact(&_bearingsFact);
    _addFact(&_sourcesFact);

    _dbmFact.setRawValue(std::numeric_limits<double>::quiet_NaN());
}

void VehicleRFFactGroup::handleMessage(Vehicle *vehicle, const mavlink_message_t &message)
{
    Q_UNUSED(vehicle);

    switch (message.msgid) {
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
        _handleNamedValueFloat(message);
        break;
    case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
        _handleDebugFloatArray(message);
        break;
    default:
        break;
    }
}

void VehicleRFFactGroup::_handleNamedValueFloat(const mavlink_message_t &message)
{
    mavlink_named_value_float_t nvf{};
    mavlink_msg_named_value_float_decode(&message, &nvf);
    const QString name = QString::fromLatin1(nvf.name, strnlen(nvf.name, sizeof(nvf.name)));

    if (name == QStringLiteral("RF_dBm")) {
        _dbmFact.setRawValue(static_cast<double>(nvf.value));
    } else if (name == QStringLiteral("RFS_SWP")) {
        // 0 tracking the leg, 1-4 sweep phases (left, right, back, return): count each new sweep
        const uint8_t state = static_cast<uint8_t>(nvf.value);
        if ((state != 0) && (_sweepFact.rawValue().toUInt() == 0)) {
            _sweepsFact.setRawValue(_sweepsFact.rawValue().toUInt() + 1);
        }
        _sweepFact.setRawValue(state);
    } else {
        return;
    }
    _setTelemetryAvailable(true);
}

void VehicleRFFactGroup::_handleDebugFloatArray(const mavlink_message_t &message)
{
    mavlink_debug_float_array_t dfa{};
    mavlink_msg_debug_float_array_decode(&message, &dfa);
    const QString name = QString::fromLatin1(dfa.name, strnlen(dfa.name, sizeof(dfa.name)));

    if (name == QStringLiteral("RF_BRG")) {
        _bearingIds.insert(static_cast<int>(dfa.data[6]));
        _bearingsFact.setRawValue(_bearingIds.count());
    } else if (name == QStringLiteral("RF_FIX")) {
        // major axis < 0 removes the source
        if (dfa.data[2] < 0) {
            _sourceIds.remove(dfa.array_id);
        } else {
            _sourceIds.insert(dfa.array_id);
        }
        _sourcesFact.setRawValue(_sourceIds.count());
    } else {
        return;
    }
    _setTelemetryAvailable(true);
}
