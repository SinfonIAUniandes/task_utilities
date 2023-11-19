ALTER TABLE "pepper_tests"
ADD COLUMN evaluated_by uuid;

ALTER TABLE "pepper_tests"
ADD CONSTRAINT fk_evaluated_by
FOREIGN KEY (evaluated_by) REFERENCES users(id);