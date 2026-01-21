import { useEffect, useRef } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import * as THREE from "three";
import type { PhysicsWorldInstance } from "../types";

export const MouseHandler = ({ worldRef }: { worldRef: React.MutableRefObject<PhysicsWorldInstance | null> }) => {
    const { camera, scene, gl } = useThree();
    const raycaster = useRef(new THREE.Raycaster());
    const mouse = useRef(new THREE.Vector2());
    const draggedObject = useRef<number | null>(null);
    const dragPlane = useRef(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0));
    const targetPoint = useRef(new THREE.Vector3());

    useEffect(() => {
        const handleDown = (e: MouseEvent) => {
            if (e.button !== 0) return; 
            const rect = gl.domElement.getBoundingClientRect();
            mouse.current.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            mouse.current.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

            raycaster.current.setFromCamera(mouse.current, camera);
            const intersects = raycaster.current.intersectObjects(scene.children, true);

            for (const hit of intersects) {
                if (hit.object.userData.physicsId !== undefined) {
                    draggedObject.current = hit.object.userData.physicsId;
                    const normal = new THREE.Vector3();
                    camera.getWorldDirection(normal);
                    dragPlane.current.setFromNormalAndCoplanarPoint(normal, hit.point);
                    // eslint-disable-next-line @typescript-eslint/no-explicit-any
                    if ((scene as any).userData.controls) (scene as any).userData.controls.enabled = false;
                    break;
                }
            }
        };

        const handleUp = () => {
            draggedObject.current = null;
            // eslint-disable-next-line @typescript-eslint/no-explicit-any
            if ((scene as any).userData.controls) (scene as any).userData.controls.enabled = true;
        };

        const handleMove = (e: MouseEvent) => {
            if (draggedObject.current === null) return;
            const rect = gl.domElement.getBoundingClientRect();
            mouse.current.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            mouse.current.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
            raycaster.current.setFromCamera(mouse.current, camera);
            raycaster.current.ray.intersectPlane(dragPlane.current, targetPoint.current);
        };

        gl.domElement.addEventListener('mousedown', handleDown);
        window.addEventListener('mouseup', handleUp);
        window.addEventListener('mousemove', handleMove);
        return () => {
            gl.domElement.removeEventListener('mousedown', handleDown);
            window.removeEventListener('mouseup', handleUp);
            window.removeEventListener('mousemove', handleMove);
        };
    }, [camera, scene, gl]);

    useFrame(() => {
        if (draggedObject.current !== null && worldRef.current) {
            const id = draggedObject.current;
            const bodyPos = worldRef.current.getBodyPosition(id);
            if (bodyPos && worldRef.current.applyForce) {
                const k = 25.0; 
                const currentPos = new THREE.Vector3(bodyPos.pos.x, bodyPos.pos.y, bodyPos.pos.z);
                const force = new THREE.Vector3().subVectors(targetPoint.current, currentPos).multiplyScalar(k);
                force.clampLength(0, 1000);
                worldRef.current.applyForce(id, force.x, force.y, force.z);
            }
        }
    });
    return null;
};
