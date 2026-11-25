#pragma once

#include <QtCore/QSet>

#include "FactGroup.h"

/// RF direction finding (compare/rf_companion.py and the RFSEARCH mode of our ArduPlane build):
/// the companion's live analyzer reading, the autopilot's sweep state and what has been found so far
class VehicleRFFactGroup : public FactGroup
{
    Q_OBJECT
    Q_PROPERTY(Fact *dbm      READ dbm      CONSTANT)
    Q_PROPERTY(Fact *sweep    READ sweep    CONSTANT)
    Q_PROPERTY(Fact *sweeps   READ sweeps   CONSTANT)
    Q_PROPERTY(Fact *bearings READ bearings CONSTANT)
    Q_PROPERTY(Fact *sources  READ sources  CONSTANT)

public:
    explicit VehicleRFFactGroup(QObject *parent = nullptr);

    Fact *dbm() { return &_dbmFact; }
    Fact *sweep() { return &_sweepFact; }
    Fact *sweeps() { return &_sweepsFact; }
    Fact *bearings() { return &_bearingsFact; }
    Fact *sources() { return &_sourcesFact; }

    // Overrides from FactGroup
    void handleMessage(Vehicle *vehicle, const mavlink_message_t &message) final;

private:
    void _handleNamedValueFloat(const mavlink_message_t &message);
    void _handleDebugFloatArray(const mavlink_message_t &message);

    Fact _dbmFact = Fact(0, QStringLiteral("dbm"), FactMetaData::valueTypeDouble);
    Fact _sweepFact = Fact(0, QStringLiteral("sweep"), FactMetaData::valueTypeUint8);
    Fact _sweepsFact = Fact(0, QStringLiteral("sweeps"), FactMetaData::valueTypeUint16);
    Fact _bearingsFact = Fact(0, QStringLiteral("bearings"), FactMetaData::valueTypeUint16);
    Fact _sourcesFact = Fact(0, QStringLiteral("sources"), FactMetaData::valueTypeUint16);

    QSet<int> _bearingIds;
    QSet<int> _sourceIds;
};
