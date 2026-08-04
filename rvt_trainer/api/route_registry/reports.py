"""Analysis, comparison, prediction, and export route declarations."""

from .types import AuthPolicy, RouteGroup, route

ROUTES = (
    route("session_summary", "GET", "/api/sessions/*/summary", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route("session_data", "GET", "/api/sessions/*/data", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route("session_compare", "GET", "/api/sessions/*/compare", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route(
        "session_analysis_status",
        "GET",
        "/api/sessions/*/analyse/status",
        RouteGroup.REPORTS,
        AuthPolicy.OPERATOR,
    ),
    route(
        "session_training_status",
        "GET",
        "/api/sessions/*/training/status",
        RouteGroup.REPORTS,
        AuthPolicy.OPERATOR,
    ),
    route("session_predict", "GET", "/api/sessions/*/predict", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route("session_analyse", "POST", "/api/sessions/*/analyse", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route("study_analysis_start", "POST", "/api/study/analysis", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route("study_analysis_list", "GET", "/api/study/analysis", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route("study_analysis_status", "GET", "/api/study/analysis/*", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route("study_analysis_cancel", "DELETE", "/api/study/analysis/*", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route("study_objective_report", "GET", "/api/study/objectives/*/report", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
    route("report_export", "GET", "/api/report/export", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
)

__all__ = ["ROUTES"]
