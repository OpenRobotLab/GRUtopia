from fastapi import APIRouter

from webui.src.routes.isaac import router as isaac_router
from webui.src.routes.model_interactive import router as model_router

router = APIRouter()
router.include_router(isaac_router)
router.include_router(model_router)
