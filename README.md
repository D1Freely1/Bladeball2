This can be detected script: using System.Collections;
using UnityEngine;

/// <summary>
/// AutoHitBall: basic "aimbot"-style hitter for educational purposes.
/// Attach to a GameObject that represents the hitter (a bat, paddle, arm, etc.).
/// It will predict the ball position, rotate to face the intercept point,
/// and apply an impulse when aligned and in range.
/// </summary>
[RequireComponent(typeof(Transform))]
public class AutoHitBall : MonoBehaviour
{
    [Header("Target")]
    public Rigidbody ballRigidbody;       // assign the ball's Rigidbody in inspector

    [Header("Prediction")]
    public float leadTime = 0.15f;        // seconds ahead to predict the ball position
    public float maxPredictionTime = 0.5f;// clamp prediction to avoid wild leads

    [Header("Hit / Range")]
    public float detectionRange = 8f;     // max distance to consider hitting
    public float alignmentAngleDeg = 20f; // angle threshold (degrees) to consider aligned
    public float hitForce = 8f;           // impulse strength applied to the ball
    public Vector3 hitOffset = Vector3.forward * 0.5f; // local offset from transform where hit is applied

    [Header("Aiming")]
    public float rotationSpeedDeg = 720f; // how fast the hitter rotates toward target

    [Header("Cooldown")]
    public float swingCooldown = 0.4f;    // time between allowed swings (seconds)

    // internal state
    bool canSwing = true;

    void Reset()
    {
        // sensible defaults when added
        leadTime = 0.15f;
        maxPredictionTime = 0.5f;
        detectionRange = 8f;
        alignmentAngleDeg = 20f;
        hitForce = 8f;
        rotationSpeedDeg = 720f;
        swingCooldown = 0.4f;
        hitOffset = Vector3.forward * 0.5f;
    }

    void Update()
    {
        if (ballRigidbody == null) return;

        // Step 1: predict ball position
        Vector3 predicted = PredictBallPosition();

        // Step 2: check range
        Vector3 worldHitPoint = transform.TransformPoint(hitOffset);
        float dist = Vector3.Distance(worldHitPoint, predicted);
        if (dist > detectionRange) return;

        // Step 3: rotate smoothly to face predicted intercept point
        AimAt(predicted);

        // Step 4: check alignment, and attempt a hit
        if (canSwing && IsAligned(predicted))
        {
            // perform hit
            ApplyHit(predicted);
            StartCoroutine(SwingCooldownRoutine());
        }
    }

    Vector3 PredictBallPosition()
    {
        // Basic linear prediction: current position + velocity * t
        // You can refine this later (gravity compensation, drag, collisions).
        float t = Mathf.Clamp(leadTime, 0f, maxPredictionTime);
        Vector3 predicted = ballRigidbody.position + ballRigidbody.velocity * t;

        // Optional: apply rough gravity compensation for vertical drop
        // predicted.y += 0.5f * Physics.gravity.y * t * t; // uncomment if you want gravity factoring

        return predicted;
    }

    void AimAt(Vector3 targetWorldPos)
    {
        Vector3 toTarget = targetWorldPos - transform.position;
        if (toTarget.sqrMagnitude < 0.0001f) return;

        // Build desired rotation looking at the intercept point, keeping object's up
        Quaternion targetRot = Quaternion.LookRotation(toTarget.normalized, transform.up);
        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRot, rotationSpeedDeg * Time.deltaTime);
    }

    bool IsAligned(Vector3 targetWorldPos)
    {
        Vector3 toTarget = targetWorldPos - transform.position;
        if (toTarget.sqrMagnitude < 0.000001f) return true;

        float angle = Vector3.Angle(transform.forward, toTarget.normalized);
        return angle <= alignmentAngleDeg;
    }

    void ApplyHit(Vector3 targetWorldPos)
    {
        if (ballRigidbody == null) return;

        // Determine where to apply the impulse: local offset converted to world point
        Vector3 hitPointWorld = transform.TransformPoint(hitOffset);

        // Compute direction from hit point to ball predicted position (or ball center)
        Vector3 dir = (ballRigidbody.position - hitPointWorld).normalized;
        if (dir.sqrMagnitude < 0.0001f) dir = transform.forward;

        // Apply an impulse at the hit point to mimic a bat/paddle strike.
        // Using AddForceAtPosition creates torque if hit off-center which looks natural.
        Vector3 impulse = dir * hitForce;
        ballRigidbody.AddForceAtPosition(impulse, hitPointWorld, ForceMode.Impulse);

        // Optional debug
        Debug.Log($"AutoHit: applied impulse {impulse} at {hitPointWorld}");
    }

    IEnumerator SwingCooldownRoutine()
    {
        canSwing = false;
        yield return new WaitForSeconds(Mathf.Max(0.001f, swingCooldown));
        canSwing = true;
    }

    // Debug visualization in the editor
    void OnDrawGizmosSelected()
    {
        // draw hit point
        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(transform.TransformPoint(hitOffset), 0.06f);

        // draw detection range
        Gizmos.color = new Color(0f, 0.5f, 1f, 0.08f);
        Gizmos.DrawWireSphere(transform.position, detectionRange);

        if (ballRigidbody != null)
        {
            Vector3 predicted = PredictBallPosition();
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(predicted, 0.08f);
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, predicted);
        }
    }
}
