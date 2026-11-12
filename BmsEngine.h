#pragma once
#include <Arduino.h>
#include <string.h>

template <uint8_t NUM_CELLS>
class BmsEngine
{
public:
    static const uint8_t MEDIAN_WINDOW = 5;

    struct Config
    {
        uint8_t  pins[NUM_CELLS];
        float    adcRefVoltage       = 3.3f;
        int      adcMaxCounts        = 4095;
        float    cellVoltageMin      = 0.20f;
        float    cellVoltageMax      = 3.20f;
        int      frozenEpsilonCounts = 1;
        uint16_t frozenSampleLimit   = 150;
        int      maxJumpCounts       = 800;
        uint8_t  jumpConfirmSamples  = 3;
    };

    void begin(const Config &cfg)
    {
        _cfg = cfg;
        for (uint8_t i = 0; i < NUM_CELLS; i++)
        {
            pinMode(_cfg.pins[i], INPUT);
            _cell[i] = CellState();
        }
        _rangeMinRaw = (int)((_cfg.cellVoltageMin / _cfg.adcRefVoltage) * _cfg.adcMaxCounts);
        _rangeMaxRaw = (int)((_cfg.cellVoltageMax / _cfg.adcRefVoltage) * _cfg.adcMaxCounts);
    }

    void update()
    {
        int     raw[NUM_CELLS];
        bool    jumpedThisSample[NUM_CELLS];
        uint8_t jumpCount = 0;

        for (uint8_t i = 0; i < NUM_CELLS; i++)
        {
            raw[i] = analogRead(_cfg.pins[i]);
            jumpedThisSample[i] = evaluateSensor(i, raw[i]);
            if (jumpedThisSample[i]) jumpCount++;
        }

        uint8_t correlationMin = (NUM_CELLS + 1) / 2;
        bool correlatedTransient = (jumpCount >= correlationMin);
        _lastTransientSuppressed = correlatedTransient;

        for (uint8_t i = 0; i < NUM_CELLS; i++)
        {
            if (jumpedThisSample[i] && !correlatedTransient)
                _cell[i].jumpConfirmCount++;
            else
                _cell[i].jumpConfirmCount = 0;

            _cell[i].sensorFault = _cell[i].outOfRange
                                 || _cell[i].frozen
                                 || (_cell[i].jumpConfirmCount >= _cfg.jumpConfirmSamples);
        }

        float socSum    = 0.0f;
        float biggestV   = -1.0f;
        float smallestV  = 1.0e9f;
        _strongestIdx = 0;
        _weakestIdx   = 0;

        for (uint8_t i = 0; i < NUM_CELLS; i++)
        {
            _cellVoltage[i] = _cell[i].filteredVoltage;
            _cellSoC[i]     = (_cellVoltage[i] / _cfg.cellVoltageMax) * 100.0f;
            socSum += _cellSoC[i];

            if (_cellVoltage[i] > biggestV)  { biggestV  = _cellVoltage[i]; _strongestIdx = i; }
            if (_cellVoltage[i] < smallestV) { smallestV = _cellVoltage[i]; _weakestIdx   = i; }
        }

        _soc = socSum / NUM_CELLS;

        _previousImbalanceMv = _imbalanceMv;
        _imbalanceMv = (biggestV - smallestV) * 1000.0f;
        _thresholdMv = adaptiveThreshold(_soc);
    }

    float   getCellVoltage(uint8_t i)   const { return _cellVoltage[i]; }
    float   getCellSoC(uint8_t i)       const { return _cellSoC[i]; }
    bool    isCellFaulted(uint8_t i)    const { return _cell[i].sensorFault; }
    bool    isCellFrozen(uint8_t i)     const { return _cell[i].frozen; }
    bool    isCellOutOfRange(uint8_t i) const { return _cell[i].outOfRange; }
    uint8_t getJumpConfirmCount(uint8_t i) const { return _cell[i].jumpConfirmCount; }

    uint8_t getStrongestCellIndex() const { return _strongestIdx; }
    uint8_t getWeakestCellIndex()   const { return _weakestIdx; }

    float getPackSoC()     const { return _soc; }
    float getImbalanceMv() const { return _imbalanceMv; }
    float getThresholdMv() const { return _thresholdMv; }

    float getImbalanceTrendMvPerSample() const { return _imbalanceMv - _previousImbalanceMv; }
    float getImbalanceTrendMvPerSec(unsigned long sampleIntervalMs) const
    {
        if (sampleIntervalMs == 0) return 0.0f;
        return getImbalanceTrendMvPerSample() * (1000.0f / (float)sampleIntervalMs);
    }

    bool anySensorFault() const
    {
        for (uint8_t i = 0; i < NUM_CELLS; i++)
            if (_cell[i].sensorFault) return true;
        return false;
    }

    bool wasTransientSuppressed() const { return _lastTransientSuppressed; }

    static uint8_t cellCount() { return NUM_CELLS; }

private:
    struct CellState
    {
        int      rawWindow[MEDIAN_WINDOW] = {0};
        uint8_t  windowIndex   = 0;
        bool     windowFilled  = false;
        int      lastRaw       = -1;

        uint16_t frozenCount   = 0;
        uint8_t  jumpConfirmCount = 0;

        bool     outOfRange = false;
        bool     frozen     = false;
        bool     sensorFault = false;

        float    filteredVoltage = 0.0f;
    };

    Config    _cfg;
    CellState _cell[NUM_CELLS];

    float _cellVoltage[NUM_CELLS] = {0};
    float _cellSoC[NUM_CELLS]     = {0};

    int _rangeMinRaw = 0;
    int _rangeMaxRaw = 4095;

    uint8_t _strongestIdx = 0;
    uint8_t _weakestIdx   = 0;

    float _soc = 0.0f;
    float _imbalanceMv = 0.0f;
    float _previousImbalanceMv = 0.0f;
    float _thresholdMv = 0.0f;
    bool  _lastTransientSuppressed = false;

    static int windowMedian(const int *buf, uint8_t n)
    {
        int sorted[MEDIAN_WINDOW];
        memcpy(sorted, buf, n * sizeof(int));
        for (uint8_t i = 1; i < n; i++)
        {
            int key = sorted[i];
            int j = (int)i - 1;
            while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
            sorted[j + 1] = key;
        }
        return sorted[n / 2];
    }

    static float adaptiveThreshold(float soc)
    {
        if (soc < 20) return 180.0f;
        else if (soc < 30) return 150.0f;
        else if (soc < 40) return 130.0f;
        else if (soc < 50) return 110.0f;
        else if (soc < 70) return 100.0f;
        else if (soc < 80) return 110.0f;
        else if (soc < 90) return 130.0f;
        else return 150.0f;
    }

    bool evaluateSensor(uint8_t i, int rawValue)
    {
        CellState &c = _cell[i];

        c.outOfRange = (rawValue < _rangeMinRaw || rawValue > _rangeMaxRaw);

        if (c.lastRaw >= 0 && abs(rawValue - c.lastRaw) <= _cfg.frozenEpsilonCounts)
            c.frozenCount++;
        else
            c.frozenCount = 0;
        c.frozen = (c.frozenCount >= _cfg.frozenSampleLimit);

        c.rawWindow[c.windowIndex] = rawValue;
        c.windowIndex = (c.windowIndex + 1) % MEDIAN_WINDOW;
        if (c.windowIndex == 0) c.windowFilled = true;

        uint8_t n = c.windowFilled ? MEDIAN_WINDOW : c.windowIndex;
        int med = (n > 0) ? windowMedian(c.rawWindow, n) : rawValue;
        c.filteredVoltage = med * _cfg.adcRefVoltage / (float)_cfg.adcMaxCounts;

        bool jumped = (c.lastRaw >= 0) && (abs(rawValue - c.lastRaw) > _cfg.maxJumpCounts);
        c.lastRaw = rawValue;
        return jumped;
    }
};