"""Session collection and persistence route declarations."""

from .types import AuthPolicy, RouteGroup, route

ROUTES = (
    route("participants_list", "GET", "/api/participants", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("participants_create", "POST", "/api/participants", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("participant_status", "PUT", "/api/participants/*", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("study_completion_matrix", "GET", "/api/study/completion-matrix", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("study_objectives", "GET", "/api/study/objectives", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("study_readiness", "GET", "/api/study/readiness", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("study_protocol", "GET|PUT", "/api/study/protocol", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("study_schedule", "GET", "/api/study/schedule", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("study_attempt_create", "POST", "/api/study/attempts", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_start", "POST", "/api/session/start", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_stop", "POST", "/api/session/stop", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_annotate", "POST", "/api/session/annotate", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_annotations", "POST", "/api/session/annotations", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("sessions_list", "GET", "/api/sessions", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_files", "GET", "/api/sessions/*/files/**", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_annotations_get", "GET", "/api/sessions/*/annotations", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_notes_get", "GET", "/api/sessions/*/notes", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_notes_put", "PUT", "/api/sessions/*/notes", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_signoff_get", "GET", "/api/sessions/*/signoff", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_signoff_put", "PUT", "/api/sessions/*/signoff", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_references_get", "GET", "/api/sessions/*/references", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_references_post", "POST", "/api/sessions/*/references", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_rr_adjudication", "POST", "/api/sessions/*/references/rr-adjudication", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_tags", "PUT", "/api/sessions/*/tags", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
    route("session_delete", "DELETE", "/api/sessions/*", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
)

__all__ = ["ROUTES"]
