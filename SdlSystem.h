#pragma once

class SdlSystem final
{
public:
    SdlSystem() = default;
    ~SdlSystem();

    SdlSystem(const SdlSystem&) = delete;
    SdlSystem& operator=(const SdlSystem&) = delete;

    [[nodiscard]] bool Initialize();
    void Destroy();

private:
    bool initialized_ = false;
};