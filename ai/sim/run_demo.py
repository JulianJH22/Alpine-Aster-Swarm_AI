"""Demo: dos brazos clasifican a la vez (arm_1 -> zone_util, arm_4 -> zone_scrap).

    python run_demo.py                      # sin ventana, imprime resultados
    python run_demo.py --gui --realtime     # ventana 3D de PyBullet a velocidad real
    python run_demo.py --render out/        # guarda fotos PNG (sin pantalla)
    python run_demo.py --serve 7777         # abre la pasarela TCP para el nucleo C++
"""
import argparse
from pathlib import Path

from swarm_sim import ManeuverCommand as C, ManeuverType as T, Pose, SimWorld
from swarm_sim.gateway import make_server


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(Path(__file__).parent / "config" / "scenario.json"))
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--realtime", action="store_true")
    ap.add_argument("--render", metavar="DIR")
    ap.add_argument("--telemetry", default="out/telemetry.jsonl")
    ap.add_argument("--serve", type=int, metavar="PORT")
    args = ap.parse_args()

    w = SimWorld(args.config, gui=args.gui, realtime=args.realtime)
    if args.serve:
        srv = make_server(w, port=args.serve)
        print(f"Pasarela escuchando en 127.0.0.1:{args.serve} (Ctrl+C para salir)")
        try:
            srv.serve_forever()
        finally:
            srv.server_close()
        return

    snap = lambda name: args.render and w.render(Path(args.render) / name)
    snap("1_inicio.png")
    a1, a4 = w.arms["arm_1"], w.arms["arm_4"]

    a1.submit(C("g1", "g1", "arm_1", T.GRASP, object_id="obj_1"))
    a4.submit(C("g4", "g4", "arm_4", T.GRASP, object_id="obj_2"))
    w.run_for(1.7)
    snap("2_agarrando.png")
    w.run_until_idle(30)

    a1.submit(C("s1", "s1", "arm_1", T.SORT, target=Pose((0.32, 0.0, 0.022))))
    a4.submit(C("s4", "s4", "arm_4", T.SORT, target=Pose((-0.32, 0.0, 0.022))))
    w.run_for(1.0)
    snap("3_clasificando.png")
    w.run_until_idle(30)
    w.run_for(0.5)
    snap("4_final.png")

    for arm, ids in ((a1, ("g1", "s1")), (a4, ("g4", "s4"))):
        for e in ids:
            r = arm.result(e)
            error_text = f"{r.error_m:.4f}" if r.error_m is not None else "N/A"
            print(f"{arm.id} {e}: {r.status.value:9s} {r.detail:4s} t={r.elapsed_s:.2f}s err={error_text} m")
    print({o.object_id: (o.zone or "-") for o in w.env.observe().objects})
    w.telemetry.save_jsonl(args.telemetry)
    print(f"t_sim={w.time:.2f}s  eventos={len(w.telemetry.events)}  muestras={len(w.telemetry.samples)}  "
          f"digest={w.telemetry.digest()[:16]}  -> {args.telemetry}")
    if args.gui:
        input("Enter para cerrar...")
    w.close()


if __name__ == "__main__":
    main()
