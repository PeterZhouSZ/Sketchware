#pragma once
#include <functional>
#include <LearningQuadrangulationsCore/learning_quadrangulations.h>

struct Notifiable : public vcg::tri::pl::MT_PatchPriorityLookupNotifiable {
    std::function<void(
        vcg::tri::pl::MT_PatchPriorityLookupNotifiable::NotificationType,
        vcg::tri::pl::count_type,
        vcg::tri::pl::count_type,
        std::chrono::nanoseconds)> func;
    void notify(
        vcg::tri::pl::MT_PatchPriorityLookupNotifiable::NotificationType notificationType,
        vcg::tri::pl::count_type nTemplatePatches,
        vcg::tri::pl::count_type nPatches,
        std::chrono::nanoseconds duration) override
    {
        func(notificationType, nTemplatePatches, nPatches, duration);
    }
};
